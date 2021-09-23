package frc2020.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.ShootingLocation;
import frc2020.statemachines.SuperstructureStateMachine;
import frc2020.statemachines.SuperstructureStateMachine.SystemState;
import frc2020.Robot;
import frc2020.RobotState;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;

public class Superstructure extends Subsystem {
    private static Superstructure mInstance;

    private static RobotState mRobotState = RobotState.getInstance();

    private static Intake mIntake = Intake.getInstance();
    private static Inventory mInventory = Inventory.getInstance();
    private static Hood mHood = Hood.getInstance();
    private static Feeder mFeeder = Feeder.getInstance();
    private static Shooter mShooter = Shooter.getInstance();
    private static Limelight mLimelight = Limelight.getInstance();

    private SuperstructureStateMachine mStateMachine = new SuperstructureStateMachine();
    private SuperstructureStateMachine.SuperstructureState mCurrentState = new SuperstructureStateMachine.SuperstructureState();

    private SuperstructureStateMachine.WantedAction mWantedAction = SuperstructureStateMachine.WantedAction.IDLE;
    private ShootingLocation.Location mWantedShootingLocation = ShootingLocation.Location.NONE;
    // ------- CHANGE THIS FOR A PERMANENT ALTITUDE OFFSET -------
    private double mAltitudeOffset = 0;

    private double mBrushOverride = 0;
    private Optional<Double> mHoodAngleOverride = Optional.empty();

    private boolean disabled = false;

    private boolean mAtRPM = false;

    private Optional<Boolean> mIntakeDeployOverride = Optional.empty();

    private Optional<Boolean> mLidDeployOverride = Optional.empty();

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {

    }

    @Override
    public void registerEnabledLoops(final ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(final double timestamp) {
                synchronized (Superstructure.this) {
                    stop();
                }
            }

            @Override
            public void onLoop(final double timestamp) {
                synchronized (Superstructure.this) {
                    if (!disabled) {
                        // This is needed so that the inventory knows if it should increment the ball
                        // counter
                        mInventory.setIntaking(systemStateIsIntaking());
                        if (getSystemState() == SuperstructureStateMachine.SystemState.BLOW) {
                            mInventory.resetBallCount();
                        } else if (getSystemState() == SuperstructureStateMachine.SystemState.SHOOT) {
                            mInventory.resetBallCount();
                        }

                        // Read current state
                        mCurrentState.intakeStalled = mIntake.isStalled();

                        mCurrentState.hood = mHood.getAngle();
                        mCurrentState.shooterRPM = mShooter.getRPM();
                        mCurrentState.shooterVoltage = mShooter.getVoltage();

                        mCurrentState.targetXAngle = mLimelight.getXAngle();
                        mCurrentState.targetValid = mLimelight.getValid();
                        mCurrentState.sensorValues = mInventory.getSensorValues();
                        mCurrentState.ballCount = mInventory.getBallCount();

                        // Get new state based on wanted actions and current state
                        var newState = mStateMachine.update(timestamp, mCurrentState, mWantedAction,
                                mWantedShootingLocation, mAltitudeOffset);

                        // Write subsystem outputs based on new state
                        if(mIntakeDeployOverride.isEmpty()) {
                            mIntake.setIntakeDeploy(newState.intakeDeploy);
                        } else {
                            mIntake.setIntakeDeploy(mIntakeDeployOverride.get());
                        }
                    
                        if (mLidDeployOverride.isEmpty()) {
                            mIntake.setLidDeploy(newState.lidDeploy);
                        }
                        else {
                            mIntake.setLidDeploy(mLidDeployOverride.get());
                        }

                        mIntake.setIntake(newState.intakeVoltage);

                        if (mBrushOverride == 0) {
                            mIntake.setIndexer(newState.brushVoltage);
                        } else {
                            mIntake.setIndexer(mBrushOverride);
                        }

                        if(mHoodAngleOverride.isEmpty()) {
                            mHood.setAngle(newState.hood);
                        } else {
                            mHood.setAngle(mHoodAngleOverride.get());
                        }
                        
                        mFeeder.setDemand(newState.feederVoltage);

                        boolean isShooterVoltage = !Double.isNaN(newState.shooterVoltage);
                        boolean isShooterRPM = !Double.isNaN(newState.shooterRPM);
                        try {
                            if (isShooterVoltage && isShooterRPM) {
                                throw new Exception("SuperstructureStateMachine exception: cannot set shooter voltage and RPM at the same time. One or both must be equal to Double.NaN"); 
                            }
                            if(Robot.disableShooter || (!isShooterVoltage && !isShooterRPM)) {
                                mShooter.setDemand(0);
                            } else if(isShooterVoltage) {
                                mShooter.setDemand(newState.shooterVoltage);
                            } else {
                                mShooter.setTargetRPM(newState.shooterRPM);
                            }
                        } catch (Exception e) {
                            e.printStackTrace();
                        }

                        if(mCurrentState.shooterRPM > newState.shooterRPM) {
                            mAtRPM = true;
                        } else {
                            mAtRPM = false;
                        }
                    }
                }
            }

            @Override
            public void onStop(final double timestamp) {
                stop();
            }
        });
    }

    public boolean getAtRPM() {
        return mAtRPM;
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    public synchronized void resetStateMachine() {
        mWantedAction = SuperstructureStateMachine.WantedAction.IDLE;
        mStateMachine.reset();
    }

    //region Wanted action setters
    public void setWantedAction(SuperstructureStateMachine.WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    public void setWantIdle() {
        mWantedAction = SuperstructureStateMachine.WantedAction.IDLE;
    }

    public void setWantIntakeOn() {
        mWantedAction = SuperstructureStateMachine.WantedAction.INTAKE_ON;
    }

    public void setWantBlow() {
        mWantedAction = SuperstructureStateMachine.WantedAction.BLOW;
    }

    public void setWantLidOverride() {
        mWantedAction = SuperstructureStateMachine.WantedAction.LID_OVERRIDE;
    }

    public void setWantEnableFlywheel() {
        mWantedAction = SuperstructureStateMachine.WantedAction.ENABLE_FLYWHEEL;
    }
    
    public void setWantAimLimelight() {
        mWantedAction = SuperstructureStateMachine.WantedAction.AIM_LIGHTLIGHT;
    }

    public void setWantAimNoLimelight() {
        mWantedAction = SuperstructureStateMachine.WantedAction.AIM_NO_LIMELIGHT;
    }

    public void setWantAimManual() {
        mWantedAction = SuperstructureStateMachine.WantedAction.AIM_MANUAL;
    }

    public void setWantShoot() {
        mWantedAction = SuperstructureStateMachine.WantedAction.SHOOT;
    }
    //endregion

    public synchronized void setWantedShootingLocation(ShootingLocation.Location wantedShootingLocation) {
        mWantedShootingLocation = wantedShootingLocation;
    }

    public synchronized void clearWantedShootingLocation() {
        setWantedShootingLocation(ShootingLocation.Location.NONE);
    }

    public synchronized void changeAltitudeOffset(double deltaAltitudeOffset) {
        mAltitudeOffset += deltaAltitudeOffset; 
    }

    public synchronized void setBrushOverride(boolean reverse) {
        mBrushOverride = reverse ? -3 : 3;
    }

    public synchronized void stopBrushOverride() {
        mBrushOverride = 0;
    }

    public synchronized void setHoodAngleOverride(double angle) {
        mHoodAngleOverride = Optional.of(angle);
    }

    public synchronized void stopHoodAngleOverride() {
        mHoodAngleOverride = Optional.empty();
    }

    public synchronized void setIntakeDeployOverride(boolean deploy) {
        mIntakeDeployOverride = Optional.of(deploy);
    }

    public synchronized void stopIntakeDeployOverride() {
        mIntakeDeployOverride = Optional.empty();
    }

    public synchronized void setLidDeployOverride(boolean deploy) {
        mLidDeployOverride = Optional.of(deploy);
    }

    public synchronized void stopLidDeployOverride() {
        mLidDeployOverride = Optional.empty();
    }

    public synchronized SuperstructureStateMachine.SystemState getSystemState() {
        return mStateMachine.getSystemState();
    }

    public synchronized boolean systemStateIsIntaking() {
        var systemState = getSystemState();
        return systemStateIsIntaking(systemState);
    }

    public synchronized boolean systemStateIsLimelight() {
        var systemState = getSystemState();
        return systemStateIsLimelight(systemState);
    }

    public static boolean systemStateIsIntaking(SuperstructureStateMachine.SystemState systemState) {
        return
            systemState == SuperstructureStateMachine.SystemState.INTAKE ||
            systemState == SuperstructureStateMachine.SystemState.INDEX ||
            systemState == SuperstructureStateMachine.SystemState.UNJAM_INTAKE;
    }

    public static boolean systemStateIsLimelight(SuperstructureStateMachine.SystemState systemState) {
        return
            systemState == SuperstructureStateMachine.SystemState.AIM_LIGHTLIGHT ||
            systemState == SuperstructureStateMachine.SystemState.SHOOT;
    }

    @Override
    public synchronized void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() { 
        SmartDashboard.putString("Wanted Action", mWantedAction.toString());
        SmartDashboard.putString("System State", getSystemState().toString());
    }
}