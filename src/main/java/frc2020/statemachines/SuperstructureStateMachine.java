package frc2020.statemachines;

import edu.wpi.first.wpilibj.Timer;
import frc2020.Constants;
import frc2020.ShootingLocation;
import frc2020.states.LEDState;
import frc2020.subsystems.Shooter;
import lib.util.DelayedBoolean;
import lib.util.LatchedBoolean;
import lib.util.Util;

public class SuperstructureStateMachine {

    // Idle
    private static final double kHoodStowAngle = 45;
    private static final double kShooterIdleVoltage = 3;

    // Intake
    private static final double kIntakeVoltage = 6.5;
    private static final double kFeederIntakeVoltage = 0; // TODO find (used to be -3)
    
    // Index
    private static final double kBrushIndexVoltage = 5;

    // Blow
    private static final double kBlowVoltage = 0;
    private static final double kBrushBlowVoltage = -5; // TODO
    private static final double kFeederBlowVoltage = 0;

    // Aim
    private static final double kBrushBallPrepVoltage = 0; // TODO
    private static final double kFeederAimBackVoltage = 0; // TODO
    private static final double kFeederAimForwardVoltage = 0; // TODO
    /** Seconds */
    private static final double kFeederAimBallsBackTime = 0; // TODO

    // Shoot
    private static final double kBrushShootVoltage = 7; // TODO
    private static final double kFeederShootVoltage = 12;
    private static final double kShooterRecoveryVoltage = 12;

    private SystemState mSystemState = SystemState.IDLE;
    private SuperstructureState mDesiredState = new SuperstructureState();

    private double mCurrentStateStartTime = 0.0;

    public enum SystemState {
        IDLE,
        INTAKE, INDEX, INDEX_EXTRA, INTAKE_FINISH, BLOW, UNJAM_INTAKE,
        ENABLE_FLYWHEEL, AIM_LIGHTLIGHT, AIM_NO_LIMELIGHT, AIM_MANUAL,
        SHOOT
    }

    public enum WantedAction {
        IDLE,
        INTAKE_ON, BLOW,
        ENABLE_FLYWHEEL, AIM_LIGHTLIGHT, AIM_NO_LIMELIGHT, AIM_MANUAL,
        SHOOT
    }

    public enum IntakeDeployWantedAction {
        IDLE, INTAKE_OVERRIDE_DOWN, INTAKE_OVERRIDE_UP
    }

    public static class SuperstructureState {
        public SuperstructureState() {
            
        }

        //Desired only
        public double intakeVoltage;
        public double brushVoltage;
        public boolean intakeDeploy;

        public double feederVoltage;

        //Current and desired
        public double hood = kHoodStowAngle;
        public double shooterRPM = Double.NaN;
        public double shooterVoltage = Double.NaN;

        //Current only
        public boolean[] sensorValues;
        public int ballCount;
        public boolean intakeStalled;

        public double targetXAngle;
        public boolean targetValid;
    }

    public synchronized void reset() {
        mSystemState = SystemState.IDLE;
    }

    public SuperstructureStateMachine() {
    
    }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    //region Main Case Structure
    public synchronized SuperstructureState update(
        double timestamp,
        SuperstructureState currentState,
        WantedAction wantedAction,
        ShootingLocation.Location wantedShootingLocation,
        double altitudeOffset)
    {
        SystemState prevState = mSystemState;
        SystemState newState = mSystemState;
        double timeInState = timestamp - mCurrentStateStartTime;
        
        switch(mSystemState) {
            case IDLE:
                newState = defaultTransitions(wantedAction, currentState);
                break;
            case INTAKE:
                newState = handleIntakeTransitions(wantedAction, currentState, timeInState);
                break;
            case INDEX:
                newState = handleIndexTransitions(wantedAction, currentState);
                break;
            case INDEX_EXTRA:
                newState = handleIndexExtraTransitions(wantedAction, currentState, timeInState);
                break;
            case INTAKE_FINISH:
                newState = handleIntakeFinishTransitions(timestamp, wantedAction, currentState);
                break;
            case BLOW:
                newState = handleBlowTransitions(wantedAction, currentState);
                break;
            case UNJAM_INTAKE:
                newState = handleUnjamIntakeTransitions(timestamp, wantedAction, currentState);
                break;
            case ENABLE_FLYWHEEL:
                newState = handleEnableFlywheelTransitions(wantedAction, currentState);
                break;
            case AIM_LIGHTLIGHT:
                newState = handleAimLimelightTransitions(wantedAction, currentState);
                break;
            case AIM_NO_LIMELIGHT:
                newState = handleAimNoLimelightTransitions(wantedAction, currentState);
                break;
            case AIM_MANUAL:
                newState = handleAimManualTransitions(wantedAction, currentState);
                break;
            case SHOOT:
                newState = handleShootTransitions(wantedAction, currentState);
                break;
        }

        mSystemState = newState;

        if(prevState != newState) {
            mCurrentStateStartTime = Timer.getFPGATimestamp();
            timeInState = 0.0;
        }

        switch(mSystemState) {
            case IDLE:
                getIdleDesiredState(currentState);
                break;
            case INTAKE:
                getIntakeDesiredState(currentState);
                break;
            case INDEX:
                getIndexDesiredState(currentState);
                break;
            case INDEX_EXTRA:
                getIndexExtraDesiredState(currentState);
                break;
            case INTAKE_FINISH:
                getIntakeFinishDesiredState(currentState);
                break;
            case BLOW:
                getBlowDesiredState(currentState);
                break;
            case UNJAM_INTAKE:
                getUnjamIntakeDesiredState(currentState);
                break;
            case ENABLE_FLYWHEEL:
                getEnableFlywheelDesiredState(currentState, wantedShootingLocation);
                break;
            case AIM_LIGHTLIGHT:
                getAimLimelightDesiredState(currentState, wantedShootingLocation, timeInState);
                break;
            case AIM_NO_LIMELIGHT:
                getAimNoLimelightDesiredState(currentState, wantedShootingLocation);
                break;
            case AIM_MANUAL:
                getAimManualDesiredState(currentState, wantedShootingLocation);
                break;
            case SHOOT:
                getShootDesiredState(currentState, wantedShootingLocation, timestamp);
                break;
        }

        return mDesiredState;
    }
    //endregion

    //region Transitions
    private SystemState handleIntakeTransitions(WantedAction wantedAction, SuperstructureState currentState, double timeInState) {
        //Remain in intake position if intaking
        if(wantedAction == WantedAction.INTAKE_ON) {
            if(currentState.intakeStalled) {
                return SystemState.UNJAM_INTAKE;
            }

            if(currentState.sensorValues[0] && currentState.ballCount == 1) {
                return SystemState.INDEX;
            }

            if(currentState.ballCount == 1 || currentState.ballCount == 2) {
                /*if(timeInState >= 1) {
                    return SystemState.INDEX_EXTRA;
                }*/
            }

            return SystemState.INTAKE;
        }
        
        return defaultTransitions(wantedAction, currentState);
    }

    LatchedBoolean latch = new LatchedBoolean();
    private SystemState handleIndexTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.INTAKE_ON) {
            if(!currentState.sensorValues[0] && currentState.sensorValues[1]) {    
                return SystemState.INTAKE;
            }

            return SystemState.INDEX;
        }

        return defaultTransitions(wantedAction, currentState);
    }

    private SystemState handleIndexExtraTransitions(WantedAction wantedAction, SuperstructureState currentState, double timeInState) {
        if(wantedAction == WantedAction.INTAKE_ON) {
            if(timeInState >= 0.65) {
                return SystemState.INTAKE_FINISH;
            } else {
                return SystemState.INDEX_EXTRA;
            }
        }

        return defaultTransitions(wantedAction, currentState);
    }

    private SystemState handleIntakeFinishTransitions(double timestamp, WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.INTAKE_ON) {
            return SystemState.INTAKE_FINISH;
        } else if(wantedAction == WantedAction.IDLE) {
            return SystemState.IDLE;
        }

        return defaultTransitions(wantedAction, currentState);
    }

    private SystemState handleBlowTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        // Remain in blow position if blowing
        if(wantedAction == WantedAction.BLOW) {
            return SystemState.BLOW;
        }
        
        return defaultTransitions(wantedAction, currentState);
    }
    
    DelayedBoolean dIntakeUnstalled;
    private SystemState handleUnjamIntakeTransitions(double timestamp, WantedAction wantedAction, SuperstructureState currentState) {
        if(dIntakeUnstalled == null) {
            dIntakeUnstalled = new DelayedBoolean(timestamp, 2000 / 1000d);
        }
        var unstalled = dIntakeUnstalled.update(timestamp, !currentState.intakeStalled);
        
        if(wantedAction == WantedAction.INTAKE_ON) {
            if(unstalled) {
                return SystemState.INTAKE;
            } else {
                return SystemState.UNJAM_INTAKE;
            }
        }
        
        return defaultTransitions(wantedAction, currentState);
    }
    
    private SystemState handleEnableFlywheelTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        return aimDefaultTransitions(wantedAction, currentState);
    }

    private SystemState handleAimLimelightTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.SHOOT) {
            firstBallShot = false;
            return SystemState.SHOOT;
        }

        return aimDefaultTransitions(wantedAction, currentState);
    }
    
    private SystemState handleAimNoLimelightTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.SHOOT) {
            firstBallShot = false;
            return SystemState.SHOOT;
        }

        return aimDefaultTransitions(wantedAction, currentState);
    }

    private SystemState handleAimManualTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.SHOOT) {
            firstBallShot = false;
            return SystemState.SHOOT;
        }
        
        return aimDefaultTransitions(wantedAction, currentState);
    }

    private SystemState handleShootTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.SHOOT) {
            return SystemState.SHOOT;
        }
        
        return aimDefaultTransitions(wantedAction, currentState);
    }

    private SystemState aimDefaultTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.AIM_LIGHTLIGHT) {
            return SystemState.AIM_LIGHTLIGHT;
        } else if(wantedAction == WantedAction.AIM_NO_LIMELIGHT)  {
            return SystemState.AIM_NO_LIMELIGHT;
        } else if(wantedAction == WantedAction.AIM_MANUAL)  {
            return SystemState.AIM_MANUAL;
        }

        return defaultTransitions(wantedAction, currentState);
    }

    private SystemState defaultTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        switch(wantedAction) {
            case IDLE:
                return SystemState.IDLE;
            case INTAKE_ON:
                return SystemState.INTAKE;
            case BLOW:
                return SystemState.BLOW;
            case ENABLE_FLYWHEEL:
                return SystemState.ENABLE_FLYWHEEL;
            case AIM_LIGHTLIGHT:
                return SystemState.IDLE;
            case AIM_NO_LIMELIGHT:
                return SystemState.IDLE;
            case AIM_MANUAL:
                return SystemState.IDLE;
            case SHOOT:
                return SystemState.IDLE;
        }

        return SystemState.IDLE;
    }
    //endregion

    //region Desired State
    private void getDefaultDesiredState(SuperstructureState currentState) {
        mDesiredState.intakeVoltage = 0;
        mDesiredState.brushVoltage = 0;
        mDesiredState.intakeDeploy = false;

        mDesiredState.feederVoltage = 0;

        mDesiredState.shooterVoltage = kShooterIdleVoltage;
        mDesiredState.shooterRPM = Double.NaN;

        mDesiredState.hood = kHoodStowAngle;
    }

    private void getIdleDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
    }

    private void getIntakeDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
        mDesiredState.hood = kHoodStowAngle;

        mDesiredState.intakeDeploy = true;
        mDesiredState.intakeVoltage = kIntakeVoltage;
        mDesiredState.feederVoltage = kFeederIntakeVoltage;
    }

    private void getIndexDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
        mDesiredState.hood = kHoodStowAngle;
        mDesiredState.intakeDeploy = true;
        mDesiredState.intakeVoltage = kIntakeVoltage;
        // Increase index speed with more balls. TODO Remove?
        //mDesiredState.brushVoltage = kBrushIndexVoltage + (currentState.ballCount * 2.0);
        mDesiredState.brushVoltage = kBrushIndexVoltage;
        mDesiredState.feederVoltage = kFeederIntakeVoltage;
    }

    private void getIndexExtraDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
        mDesiredState.hood = kHoodStowAngle;
        mDesiredState.intakeDeploy = true;
        mDesiredState.intakeVoltage = kIntakeVoltage;
        mDesiredState.brushVoltage = 4;
        mDesiredState.feederVoltage = kFeederIntakeVoltage;
    }

    private void getIntakeFinishDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
    }
    
    private void getBlowDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
        mDesiredState.hood = kHoodStowAngle;

        mDesiredState.intakeDeploy = true;
        mDesiredState.intakeVoltage = kBlowVoltage;
        mDesiredState.brushVoltage = currentState.sensorValues[0] ? 0 : kBrushBlowVoltage;
        mDesiredState.feederVoltage = kFeederBlowVoltage;
    }

    private void getUnjamIntakeDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
        mDesiredState.hood = kHoodStowAngle;

        mDesiredState.intakeDeploy = true;
        mDesiredState.intakeVoltage = kBlowVoltage;
        mDesiredState.feederVoltage = kFeederIntakeVoltage;
    }

    private void getEnableFlywheelDesiredState(SuperstructureState currentState, ShootingLocation.Location wantedShootingLocation) {
        getDefaultDesiredState(currentState);
        
        mDesiredState.hood = kHoodStowAngle;

        mDesiredState.shooterVoltage = Double.NaN;
        mDesiredState.shooterRPM = wantedShootingLocation.getShooterRPM();
        mDesiredState.hood = wantedShootingLocation.getHoodAngle();

        mDesiredState.feederVoltage = kFeederAimBackVoltage;
    }

    private void getAimLimelightDesiredState(SuperstructureState currentState, ShootingLocation.Location wantedShootingLocation, double timeInState) {
        getDefaultDesiredState(currentState);
        getAimManualDesiredState(currentState, wantedShootingLocation);

        mDesiredState.brushVoltage = timeInState >= kFeederAimBallsBackTime ? 
            0 : kBrushBallPrepVoltage;
        mDesiredState.feederVoltage = kFeederAimForwardVoltage;
    }

    private void getAimNoLimelightDesiredState(SuperstructureState currentState, ShootingLocation.Location wantedShootingLocation) {
        //TODO Possibly implement (not really needed since the challenges are at home)
    }
    
    private void getAimManualDesiredState(SuperstructureState currentState, ShootingLocation.Location wantedShootingLocation) {
        getDefaultDesiredState(currentState);
        
        mDesiredState.shooterVoltage = Double.NaN;
        mDesiredState.shooterRPM = wantedShootingLocation.getShooterRPM();
        mDesiredState.hood = wantedShootingLocation.getHoodAngle();
    }

    private boolean firstBallShot = false;
    private double beforeRecoveredTimestamp = 0;
    private void getShootDesiredState(SuperstructureState currentState, ShootingLocation.Location wantedShootingLocation, double timestamp) {
        getDefaultDesiredState(currentState);

        mDesiredState.feederVoltage = kFeederShootVoltage;
        
        double rpmToRecover = Constants.kRapidFire ? 100 : 500;
        //double brushFeedRate = Constants.kRapidFire ? kBrushRapidShootVoltage : kBrushShootVoltage;
        double brushFeedRate = Constants.kRapidFire ? 7 : 5;
        double rpmAcceptableError = Constants.kRapidFire ? 200 : 20;

        if(wantedShootingLocation.getShooterRPM() - currentState.shooterRPM >= rpmToRecover)  {
            // If the target shooting RPM is 50 RPM greater than the current, set shooter to full power to recover faster
            mDesiredState.shooterVoltage = kShooterRecoveryVoltage;
            mDesiredState.shooterRPM = Double.NaN;
        } else {
            // Otherwise, continue running at the target shooting RPM
            mDesiredState.shooterVoltage = Double.NaN;
            mDesiredState.shooterRPM = wantedShootingLocation.getShooterRPM();
        }

        if(wantedShootingLocation.getShooterRPM() - currentState.shooterRPM >= 200) firstBallShot = true;
        
        boolean recovered = Util.epsilonEquals(wantedShootingLocation.getShooterRPM(), currentState.shooterRPM, rpmAcceptableError);
        if(!recovered) {
            beforeRecoveredTimestamp = timestamp;
        }

        if((timestamp - beforeRecoveredTimestamp >= 1.0 && !Constants.kRapidFire) || (recovered && Constants.kRapidFire)) {
            mDesiredState.brushVoltage = firstBallShot ? 7 : brushFeedRate;
        } else {
            mDesiredState.brushVoltage = 0;
        }

        mDesiredState.hood = wantedShootingLocation.getHoodAngle();
    }
}