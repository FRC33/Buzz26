package frc2020.subsystems;

import static frc2020.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc2020.Constants;
import frc2020.RobotState;
import lib.Kinematics;
import lib.control.Lookahead;
import lib.control.Path;
import lib.control.PathFollower;
import lib.drivers.BuzzPigeon;
import lib.drivers.BuzzTalonFX;
import lib.drivers.TalonFXFactory;
import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Twist2d;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;
import lib.util.DriveSignal;
import lib.util.LatchedBoolean;
import lib.util.LeadLagFilter;
import lib.util.SynchronousPIDF;
import lib.util.TorqueLimit;
import lib.util.Util;

public class Drive extends Subsystem {
    private static Drive mInstance;

    private RobotState mRobotState = RobotState.getInstance();
    private Limelight mLimelight = Limelight.getInstance();

    // Devices
    private BuzzTalonFX mLeftLeader, mLeftFollower, mRightLeader, mRightFollower;
    private BuzzPigeon mGyro;

    // Controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;
    private LeadLagFilter mYawControlFilter;
    private SynchronousPIDF mAutoSteerPIDF = new SynchronousPIDF(kAutoSteerKp, kAutoSteerKi, kAutoSteerKd);

    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
    }

    public synchronized static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // Initalize subsystem devices
        mLeftLeader = TalonFXFactory.createDefaultTalon(kLeftDriveLeaderId);
        mLeftFollower = TalonFXFactory.createPermanentFollowerTalon(kLeftDriveFollowerId, kLeftDriveLeaderId);
        mRightLeader = TalonFXFactory.createDefaultTalon(kRightDriveLeaderId);
        mRightFollower = TalonFXFactory.createPermanentFollowerTalon(kRightDriveFollowerId, kRightDriveLeaderId);

        boolean invert = false;
        configDriveMotor(mLeftLeader, invert);
        configDriveMotor(mLeftFollower, invert);
        configDriveMotor(mRightLeader, !invert);
        configDriveMotor(mRightFollower, !invert);

        mGyro = new BuzzPigeon();
        mGyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10);

        mYawControlFilter = new LeadLagFilter(Timer.getFPGATimestamp(), kYawLead, kYawLag);

        mAutoSteerPIDF.setMaxAbsoluteOutput(kAutoSteerMaxOutput);
    }

    private void configDriveMotor(BuzzTalonFX motor, boolean invert) {
        motor.setInverted(invert);
        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(12);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setBuzzTalonScalars(kDriveGearReduction, kDriveWheelDiameterInches, kFalconCPR);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double leftSupplyVoltage;
        public double rightSupplyVoltage;
        public double leftPos;
        public double rightPos;
        public double leftRPS;
        public double rightRPS;
        public double leftVel;
        public double rightVel;
        public Rotation2d heading = Rotation2d.identity();
        public double yaw;
        public double yawRate;

        public double leftLeaderTemperature;
        public double leftFollowerTemperature;
        public double rightLeaderTemperature;
        public double rightFollowerTemperature;

        // OUTPUTS
        public double leftPercent;
        public double rightPercent;
    }

    double lastTimestamp = 0;
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        
        double lastYaw = mPeriodicIO.yaw;

        mPeriodicIO.leftSupplyVoltage = Util.avg(mLeftLeader.getBusVoltage(), mLeftFollower.getBusVoltage());
        mPeriodicIO.rightSupplyVoltage = Util.avg(mRightLeader.getBusVoltage(), mRightFollower.getBusVoltage());

        mPeriodicIO.leftPos = Util.avg(mLeftLeader.getSurfacePos(), mLeftFollower.getSurfacePos());
        mPeriodicIO.rightPos = Util.avg(mRightLeader.getSurfacePos(), mRightFollower.getSurfacePos());

        mPeriodicIO.leftRPS = Util.avg(mLeftLeader.getMotorRPS(), mLeftFollower.getMotorRPS());
        mPeriodicIO.rightRPS = Util.avg(mRightLeader.getMotorRPS(), mRightFollower.getMotorRPS());

        mPeriodicIO.leftVel = Util.avg(mLeftLeader.getSurfaceVel(), mLeftFollower.getSurfaceVel());
        mPeriodicIO.rightVel = Util.avg(mRightLeader.getSurfaceVel(), mRightFollower.getSurfaceVel());

        mPeriodicIO.yaw = mGyro.getRawYawZeroed();
        mPeriodicIO.heading = Rotation2d.fromDegrees(mPeriodicIO.yaw);
        mPeriodicIO.yawRate = (mPeriodicIO.yaw - lastYaw) 
            / (mPeriodicIO.timestamp - lastTimestamp);

        mPeriodicIO.leftLeaderTemperature = mLeftLeader.getTemperature();
        mPeriodicIO.leftFollowerTemperature = mLeftFollower.getTemperature();
        mPeriodicIO.rightLeaderTemperature = mRightLeader.getTemperature();
        mPeriodicIO.rightFollowerTemperature = mRightFollower.getTemperature();

        lastTimestamp = mPeriodicIO.timestamp;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output
        mLeftLeader.set(ControlMode.PercentOutput, mPeriodicIO.leftPercent);
        mRightLeader.set(ControlMode.PercentOutput, mPeriodicIO.rightPercent);
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    //stop();
                    //setBrakeMode(true);
                    mYawControlFilter.reset(timestamp);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    //handleFaults();
                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case PATH_FOLLOWING:
                            if (mPathFollower != null) {
                                updatePathFollower(timestamp);
                            }
                            break;
                        default:
                            System.out.println("unexpected drive control state: " + mDriveControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                //stopLogging();
            }
        });
    }

    // region TeleOp control
    public void setBuzzDrive(double throttle, double wheel, boolean spinLeft, boolean spinRight, boolean aim) {
        setBuzzDrive(throttle, wheel, spinLeft, spinRight, aim, false);
    }

    public void setBuzzDrive(double throttle, double wheel, boolean spinLeft, boolean spinRight, boolean aim, boolean shooting) {
        var leftTorqueLimit = getTorqueLimit(mPeriodicIO.leftRPS / kFalconKV);
        var rightTorqueLimit = getTorqueLimit(mPeriodicIO.rightRPS / kFalconKV);
        var throttleTorqueLimit = leftTorqueLimit.limit(rightTorqueLimit);

        var throttleDemand = 0d;
        if(throttle > 0) {
            throttleDemand = throttle * throttleTorqueLimit.getMax();
        } else {
            throttleDemand = -throttle * throttleTorqueLimit.getMin();
        }

        var wheelDemand = 0d;
        if(Math.abs(throttle) >= kQuickTurnEnableThrottle) {
            //Speed turn (arc)
            wheelDemand = wheel *
                getAverageDriveVelocityMagnitude() *
                (Intake.getInstance().isIntakeDeployed() ? kSpeedTurnGainIntaking : kSpeedTurnGain);
        } else {
            //Quick turn (turn in place)
            var torque = 0d;
            if(wheel > 0) {
                torque = Math.min(Math.abs(leftTorqueLimit.getMax()), Math.abs(rightTorqueLimit.getMin()));
            } else {
                torque = Math.min(Math.abs(leftTorqueLimit.getMin()), Math.abs(rightTorqueLimit.getMax()));
            }
            wheelDemand = wheel * torque * kQuickTurnGain;
        }

        double autoSteerCorrection = autoSteer(Timer.getFPGATimestamp(), aim, shooting);
        if(aim && mLimelight.getValid()) {
            wheelDemand = -autoSteerCorrection;
        }

        DriveSignal yawControledSignal = yawControl(new DriveSignal(
            throttleDemand + wheelDemand, 
            throttleDemand - wheelDemand
        ));

        //TODO Change getSupplyVoltage to 12.0
        setOpenLoop(new DriveSignal(
            (yawControledSignal.getLeft() + (spinLeft ? 6 : 0)) / 12.0,
            (yawControledSignal.getRight() + (spinRight ? 6 : 0)) / 12.0
        ));
    }
    
    private TorqueLimit getTorqueLimit(double backEMF) {
        var currentTorqueLimit = kDriveCurrentLimit * kFalconKA;
        var veff = getSupplyVoltage();

        var min = backEMF > 0 ? 0 : backEMF;
        var max = backEMF > 0 ? backEMF : 0;
        min -= currentTorqueLimit;
        max += currentTorqueLimit;
        if(min < -veff) min = -veff; 
        if(max > veff) max = veff; 

        return new TorqueLimit(min, max);
    }

    /**
     * @return Voltage to apply to right side
     */
    LatchedBoolean mTargetLatchedBoolean = new LatchedBoolean();
    Rotation2d mTarget = new Rotation2d();
    double mAutoSteerError = 0;
    double mAutoSteerCorrection = 0;
    private double autoSteer(double timestamp, boolean target, boolean shooting) {
        if(mTargetLatchedBoolean.update(target)) mAutoSteerPIDF.resetIntegrator();

        if(target && mLimelight.getValid()) {
            double xAngle = mLimelight.getXAngle();
            Rotation2d targetRotation = Rotation2d.fromDegrees(-xAngle);
            Rotation2d pastFieldToVehicleRotation = 
                mRobotState.getFieldToVehicle(timestamp - (mLimelight.getLatency() / 1000d)).getRotation();
            Rotation2d fieldToVehicleRotation = mRobotState.getFieldToVehicle(timestamp).getRotation();

            // Do not change the target while shooting (the balls obscure the vision target)
            if(!shooting) {
                mTarget = pastFieldToVehicleRotation.rotateBy(targetRotation);
            }
            mAutoSteerError = mTarget.rotateBy(fieldToVehicleRotation.inverse()).getRadians();
            if(Math.abs(mAutoSteerError) > Units.degreesToRadians(kAutoSteerKiZone)) {
                mAutoSteerPIDF.resetIntegrator();
            }

            mAutoSteerCorrection = mAutoSteerPIDF.calculate(-mAutoSteerError);

            final double autoSteerMinCorrection = 0.4;
            if(!shooting && Math.abs(mAutoSteerCorrection) < autoSteerMinCorrection && Math.abs(mAutoSteerError) > Units.degreesToRadians(0.3)) {
                if(mAutoSteerError > 0) {
                    mAutoSteerCorrection = autoSteerMinCorrection;
                } else {
                    mAutoSteerCorrection = -autoSteerMinCorrection;
                }
            }
            
            return mAutoSteerCorrection;
        } else {
            mAutoSteerCorrection = 0;
            return mAutoSteerCorrection;
        }
    }

    private DriveSignal yawControl(DriveSignal driveSignal) {
        double volts = -(driveSignal.getLeft() - driveSignal.getRight());
        double estimatedVelocity = volts * kFalconKV * kDriveGearReduction * kDriveWheelDiameterInches * Math.PI;
        double estimatedYawRate = (estimatedVelocity / kDriveWheelTrackWidthInches) / (2 * Math.PI);
        double controlYawRate = estimatedYawRate * kYawFactor;
        double filteredYawRate = mYawControlFilter.update(mPeriodicIO.timestamp, mPeriodicIO.yawRate);
        double adjust = Math.min(Math.max(-kYawMaxCorrection, (filteredYawRate - controlYawRate) * kYawControlGain), kYawMaxCorrection);

        return new DriveSignal(driveSignal.getLeft() + adjust, driveSignal.getRight() - adjust);
    }
    // endregion

    // region Getters
    public double getLeftPosition() {
        return mPeriodicIO.leftPos;
    }

    public double getRightPosition() {
        return mPeriodicIO.rightPos;
    }

    public double getLeftVelocity() {
        return mPeriodicIO.leftVel;
    }

    public double getRightVelocity() {
        return mPeriodicIO.rightVel;
    }

    public Rotation2d getHeading() {
        return mPeriodicIO.heading;
    }

    public double getAutoSteerError() {
        return mAutoSteerError;
    }

    public double getLeftLinearVelocity() {
        return mPeriodicIO.leftVel;
    }

    public double getRightLinearVelocity() {
        return mPeriodicIO.rightVel;
    }

    public double getAverageDriveVelocityMagnitude() {
        return Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity()) / 2.0;
    }

    public double getSupplyVoltage() {
        return Util.avg(mPeriodicIO.leftSupplyVoltage, mPeriodicIO.rightSupplyVoltage);
    }
    // endregion

    public void resetGryo() {
        mGyro.reset();
    }

    public synchronized void setBraked(boolean braked) {
        mLeftLeader.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
        mLeftFollower.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
        mRightLeader.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
        mRightFollower.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized void setOpenLoop() {
        setOpenLoop(new DriveSignal(0, 0));
    }

    // region Path following
    /**
     * Configure for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal driveSignal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            System.out.println("switching to open loop");
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.leftPercent = driveSignal.getLeft();
        mPeriodicIO.rightPercent = driveSignal.getRight();
    }

    public synchronized void setVelocity(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            System.out.println("switching to path following");
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }

        mPeriodicIO.leftPercent = signal.getLeft();
        mPeriodicIO.rightPercent = signal.getRight();
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(
                    new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead, Constants.kMinLookAheadSpeed,
                            Constants.kMaxLookAheadSpeed),
                    Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                    Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                    Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                    Constants.kPathFollowingProfileKs, Constants.kPathFollowingMaxVel,
                    Constants.kPathFollowingMaxAccel, Constants.kPathFollowingGoalPosTolerance,
                    Constants.kPathFollowingGoalVelTolerance, Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setBuzzDrive(0, 0, false, false, false);
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            RobotState robot_state = RobotState.getInstance();
            Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                    robot_state.getPredictedVelocity().dx);
            if (!mPathFollower.isFinished()) {
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                //setVelocity(setpoint, new DriveSignal(0, 0));
                setVelocity(setpoint);
            } else {
                if (!mPathFollower.isForceFinished()) {
                    //setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
                    setOpenLoop(new DriveSignal(0, 0));
                }
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }
    // endregion

    @Override
    public void zeroSensors() {
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
        SmartDashboard.putNumber("Velocity", Util.avg(mPeriodicIO.leftVel, mPeriodicIO.rightVel));
        SmartDashboard.putNumber("Left Position", getLeftPosition());
        SmartDashboard.putNumber("Right Position", getRightPosition());
        SmartDashboard.putNumber("Drive Supply Voltage", getSupplyVoltage());
        SmartDashboard.putNumber("Auto Steer Heading Target", mTarget.getDegrees());
        SmartDashboard.putNumber("Auto Steer Correction V", mAutoSteerCorrection);
        SmartDashboard.putNumber("Auto Steer Error Deg", (mAutoSteerError * 180) / Math.PI);

        SmartDashboard.putNumber("Left Leader Temp", mPeriodicIO.leftLeaderTemperature);
        SmartDashboard.putNumber("Left Follower Temp", mPeriodicIO.leftFollowerTemperature);
        SmartDashboard.putNumber("Right Leader Temp", mPeriodicIO.rightLeaderTemperature);
        SmartDashboard.putNumber("Right Follower Temp", mPeriodicIO.rightFollowerTemperature);

        if(mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            var debug = mPathFollower.getDebug();
            SmartDashboard.putNumber("lx", debug.lookahead_point_x);
            SmartDashboard.putNumber("ly", debug.lookahead_point_y);
            SmartDashboard.putNumber("lv", debug.lookahead_point_velocity);
            SmartDashboard.putNumber("along track error", debug.along_track_error);
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}