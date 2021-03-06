package frc2020.statemachines;

import edu.wpi.first.wpilibj.Timer;
import frc2020.ShootingLocation;
import frc2020.states.LEDState;
import frc2020.subsystems.Shooter;
import lib.util.DelayedBoolean;
import lib.util.Util;

public class SuperstructureStateMachine {

    // Idle
    private static final double kHoodStowAngle = 58;
    private static final double kShooterIdleVoltage = 3;

    // Intake
    private static final double kIntakeVoltage = 11;
    private static final double kInfeederVoltage = 5;
    private static final double kKickerIntakeVoltage = -3;
    
    // Index
    private static final double kBrushIndexVoltage = 5;

    // Blow
    private static final double kBlowVoltage = -11;
    private static final double kBrushBlowVoltage = -13;
    private static final double kKickerBlowVoltage = -5;

    // Aim
    private static final double kBrushBallPrepVoltage = -7;
    private static final double kKickerAimBackVoltage = -5;
    private static final double kKickerAimForwardVoltage = 12;
    /** Seconds */
    private static final double kKickerAimBallsBackTime = 0.2;

    // Shoot
    private static final double kBrushShootVoltage = 7.5;
    private static final double kKickerShootVoltage = 12;
    private static final double kShooterRecoveryVoltage = 12;

    private SystemState mSystemState = SystemState.IDLE;
    private SuperstructureState mDesiredState = new SuperstructureState();

    private double mCurrentStateStartTime = 0.0;

    public enum SystemState {
        IDLE,
        INTAKE, INDEX, INTAKE_FINISH, BLOW, UNJAM_INTAKE,
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
        public double infeederVoltage;
        public double brushVoltage;
        public boolean intakeDeploy;

        public double kickerVoltage;

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
                newState = handleIntakeTransitions(wantedAction, currentState);
                break;
            case INDEX:
                newState = handleIndexTransitions(wantedAction, currentState);
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
                getShootDesiredState(currentState, wantedShootingLocation, timeInState);
                break;
        }

        return mDesiredState;
    }
    //endregion

    //region Transitions
    private SystemState handleIntakeTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        //Remain in intake position if intaking
        if(wantedAction == WantedAction.INTAKE_ON) {
            if(currentState.intakeStalled) {
                return SystemState.UNJAM_INTAKE;
            }

            if(currentState.sensorValues[0]) {
                return SystemState.INDEX;
            } else {
                return SystemState.INTAKE;
            }
        }
        
        return defaultTransitions(wantedAction, currentState);
    }

    private SystemState handleIndexTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.INTAKE_ON) {
            int stopSensor = currentState.ballCount;
            if(stopSensor < 1) stopSensor = 1; // Ensure that balls do not stop at first sensor
            if((currentState.sensorValues[stopSensor] || currentState.sensorValues[4]) && !currentState.sensorValues[0]) {
                if(currentState.ballCount >= 3) {
                    return SystemState.INTAKE_FINISH;
                } else {
                    return SystemState.INTAKE;
                }
            } else {
                return SystemState.INDEX;
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
            return SystemState.SHOOT;
        }

        return aimDefaultTransitions(wantedAction, currentState);
    }
    
    private SystemState handleAimNoLimelightTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.SHOOT) {
            return SystemState.SHOOT;
        }

        return aimDefaultTransitions(wantedAction, currentState);
    }

    private SystemState handleAimManualTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if(wantedAction == WantedAction.SHOOT) {
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
        mDesiredState.infeederVoltage = 0;
        mDesiredState.brushVoltage = 0;
        mDesiredState.intakeDeploy = false;

        mDesiredState.kickerVoltage = 0;

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
        mDesiredState.infeederVoltage = kInfeederVoltage;
        mDesiredState.kickerVoltage = kKickerIntakeVoltage;
    }

    private void getIndexDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
        mDesiredState.hood = kHoodStowAngle;

        mDesiredState.intakeDeploy = true;
        mDesiredState.intakeVoltage = kIntakeVoltage;
        mDesiredState.infeederVoltage = kInfeederVoltage;
        // Increase index speed with more balls. TODO Remove?
        mDesiredState.brushVoltage = kBrushIndexVoltage + (currentState.ballCount * 2.0);
        mDesiredState.kickerVoltage = kKickerIntakeVoltage;
    }

    private void getIntakeFinishDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
    }
    
    private void getBlowDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
        mDesiredState.hood = kHoodStowAngle;

        mDesiredState.intakeDeploy = true;
        mDesiredState.intakeVoltage = kBlowVoltage;
        mDesiredState.infeederVoltage = kBlowVoltage;
        mDesiredState.brushVoltage = kBrushBlowVoltage;
        mDesiredState.kickerVoltage = kKickerBlowVoltage;
    }

    private void getUnjamIntakeDesiredState(SuperstructureState currentState) {
        getDefaultDesiredState(currentState);
        mDesiredState.hood = kHoodStowAngle;

        mDesiredState.intakeDeploy = true;
        mDesiredState.intakeVoltage = kBlowVoltage;
        mDesiredState.infeederVoltage = kInfeederVoltage;
        mDesiredState.kickerVoltage = kKickerIntakeVoltage;
    }

    private void getEnableFlywheelDesiredState(SuperstructureState currentState, ShootingLocation.Location wantedShootingLocation) {
        getDefaultDesiredState(currentState);
        
        mDesiredState.hood = kHoodStowAngle;

        mDesiredState.shooterVoltage = Double.NaN;
        mDesiredState.shooterRPM = wantedShootingLocation.getShooterRPM();
        mDesiredState.hood = wantedShootingLocation.getHoodAngle();

        mDesiredState.kickerVoltage = kKickerAimBackVoltage;
    }

    private void getAimLimelightDesiredState(SuperstructureState currentState, ShootingLocation.Location wantedShootingLocation, double timeInState) {
        getDefaultDesiredState(currentState);
        getAimManualDesiredState(currentState, wantedShootingLocation);

        mDesiredState.brushVoltage = timeInState >= kKickerAimBallsBackTime ? 
            0 : kBrushBallPrepVoltage;
        mDesiredState.kickerVoltage = kKickerAimForwardVoltage;
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

    private void getShootDesiredState(SuperstructureState currentState, ShootingLocation.Location wantedShootingLocation, double timeInState) {
        getDefaultDesiredState(currentState);

        mDesiredState.brushVoltage = kBrushShootVoltage;
        mDesiredState.kickerVoltage = kKickerShootVoltage;
        
        if(wantedShootingLocation.getShooterRPM() - currentState.shooterRPM >= 50) {
            // If the target shooting RPM is 50 RPM greater than the current, set shooter to full power to recover faster
            mDesiredState.shooterVoltage = kShooterRecoveryVoltage;
            mDesiredState.shooterRPM = Double.NaN;
        } else {
            // Otherwise, continue running at the target shooting RPM
            mDesiredState.shooterVoltage = Double.NaN;
            mDesiredState.shooterRPM = wantedShootingLocation.getShooterRPM();
        }

        mDesiredState.hood = wantedShootingLocation.getHoodAngle();
    }
}