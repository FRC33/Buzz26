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

    private SwerveModule[] mModules = new SwerveModule[4];

    // Devices
    private BuzzPigeon mGyro;

    // Controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;

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

        mModules[0] = new SwerveModule(Constants.kFrontRightModuleConstants);
        mModules[1] = new SwerveModule(Constants.kFrontLeftModuleConstants);
        mModules[2] = new SwerveModule(Constants.kBackLeftModuleConstants);
        mModules[3] = new SwerveModule(Constants.kBackRightModuleConstants);

        // Initalize subsystem devices
        mGyro = new BuzzPigeon();
        mGyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;

        public Rotation2d heading = Rotation2d.identity();
        public double yaw;
        public double yawRate;

        // OUTPUTS
    }

    double lastTimestamp = 0;
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        
        double lastYaw = mPeriodicIO.yaw;

        mPeriodicIO.yaw = mGyro.getRawYawZeroed();
        mPeriodicIO.heading = Rotation2d.fromDegrees(mPeriodicIO.yaw);
        mPeriodicIO.yawRate = (mPeriodicIO.yaw - lastYaw) 
            / (mPeriodicIO.timestamp - lastTimestamp);
        
        lastTimestamp = mPeriodicIO.timestamp;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    //stop();
                    //setBrakeMode(true);
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

    public void setTeleOpInputs(double throttle, double strafe, double wheel) {

    }

    // region Getters
    public Rotation2d getHeading() {
        return mPeriodicIO.heading;
    }
    // endregion

    public void resetGryo() {
        mGyro.reset();
    }

    public synchronized void setBraked(boolean braked) {

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
    }

    public synchronized void setVelocity(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            System.out.println("switching to path following");
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
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
            setTeleOpInputs(0, 0, 0);
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