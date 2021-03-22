package frc2020.subsystems;

import static frc2020.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.util.Units;
import frc2020.Constants;
import frc2020.RobotState;
import lib.Kinematics;
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

    private Limelight mLimelight = Limelight.getInstance();

    // Devices
    private SwerveModule[] mModules = new SwerveModule[4];
    private BuzzPigeon mGyro;

    // Controllers
    private SwerveDriveOdometry mSwerveDriveOdometry;

    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;

    private boolean mDisabled = true;
    private boolean mFieldCentric = true;

    private State mState = new State();

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

        mSwerveDriveOdometry = new SwerveDriveOdometry(kSwerveKinematics, edu.wpi.first.wpilibj.geometry.Rotation2d.fromDegrees(0));
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;

        public Rotation2d heading = Rotation2d.identity();
        public double yaw;
        public double yawRate;

        // OUTPUTS
        public SwerveModuleState[] swerveModuleStates;
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
        if(!mDisabled) {
            if(mPeriodicIO.swerveModuleStates == null) {
                for(SwerveModule module : mModules) {
                    module.disable();
                }
            } else {
                for(int i = 0; i < 4; i++) {
                    mModules[i].setVelocity(mPeriodicIO.swerveModuleStates[i].speedMetersPerSecond);
                    mModules[i].setAngle(mPeriodicIO.swerveModuleStates[i].angle.getDegrees());
                }
            }
        }
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
                    mSwerveDriveOdometry.update(getHeadingWPI(),
                        mModules[0].getModuleState(),
                        mModules[1].getModuleState(),
                        mModules[2].getModuleState(),
                        mModules[3].getModuleState());

                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case PATH_FOLLOWING:
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
        double vx = throttle * kDriveMaxLinearVelocity;
        double vy = -strafe * kDriveMaxLinearVelocity;
        double omega = -wheel * kDriveMaxAngularVelocity;

        ChassisSpeeds speeds;
        if(mFieldCentric) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeadingWPI());
        } else {
            speeds = new ChassisSpeeds(vx, vy, omega);
        }

        SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(speeds);

        mPeriodicIO.swerveModuleStates = moduleStates;
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        mPeriodicIO.swerveModuleStates = moduleStates;
    }

    public SwerveModule[] getSwerveModules() {
        return mModules;
    }

    // region Getters
    public Rotation2d getHeading() {
        return mPeriodicIO.heading;
    }

    public edu.wpi.first.wpilibj.geometry.Rotation2d getHeadingWPI() {
        return new edu.wpi.first.wpilibj.geometry.Rotation2d(mPeriodicIO.heading.getRadians());
    }

    public edu.wpi.first.wpilibj.geometry.Pose2d getPoseWPI() {
        return mSwerveDriveOdometry.getPoseMeters();
    }
    // endregion

    public void resetGyro() {
        mGyro.reset();
    }

    public void resetOdometry() {
        mSwerveDriveOdometry.resetPosition(
            new edu.wpi.first.wpilibj.geometry.Pose2d(0, 0, edu.wpi.first.wpilibj.geometry.Rotation2d.fromDegrees(0)), 
            getHeadingWPI()
        );
    }
    public void resetOdometry(edu.wpi.first.wpilibj.geometry.Pose2d pose) {
        mSwerveDriveOdometry.resetPosition(
            pose, 
            getHeadingWPI()
        );
    }

    public void setTrajectoryState(State state) {
        mState = state;
    }

    public synchronized void setDisabled(boolean disabled) {
        mDisabled = disabled;
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
        SmartDashboard.putNumber("x", Units.metersToInches(getPoseWPI().getX()));
        SmartDashboard.putNumber("y", Units.metersToInches(getPoseWPI().getY()));
        SmartDashboard.putNumber("theta", getPoseWPI().getRotation().getDegrees());

        var pose = mState.poseMeters;
        SmartDashboard.putNumber("lx", Units.metersToInches(pose.getX()));
        SmartDashboard.putNumber("ly", Units.metersToInches(pose.getY()));
        SmartDashboard.putNumber("lv", pose.getRotation().getDegrees());
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}