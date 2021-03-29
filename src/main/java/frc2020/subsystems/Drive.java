package frc2020.subsystems;

import static frc2020.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Units;
import frc2020.Constants;
import frc2020.RobotState;
import lib.Kinematics;
import lib.drivers.BuzzPigeon;
import lib.drivers.BuzzTalonFX;
import lib.drivers.BuzzXboxController;
import lib.drivers.TalonFXFactory;
import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.geometry.Twist2d;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;
import lib.util.DriveSignal;
import lib.util.LatchedBoolean;
import lib.util.LeadLagFilter;
import lib.util.ReflectingCSVWriter;
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
    private ProfiledPIDController mSteerController =
        //new ProfiledPIDController(0.02, 0, 0, new Constraints(75 / 6, (75 / 6)));
        new ProfiledPIDController(0.0, 0, 0, new Constraints(75 / 6, (75 / 6)));
    private LeadLagFilter mYawControlFilter;

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

        mYawControlFilter = new LeadLagFilter(Timer.getFPGATimestamp(), kYawLead, kYawLag);
        mSwerveDriveOdometry = new SwerveDriveOdometry(kSwerveKinematics, edu.wpi.first.wpilibj.geometry.Rotation2d.fromDegrees(0));
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;

        public double vx;
        public double vy;
        public double omega;

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

        var speeds = kSwerveKinematics.toChassisSpeeds(
            mModules[0].getModuleState(),
            mModules[1].getModuleState(),
            mModules[2].getModuleState(),
            mModules[3].getModuleState());
        
        mPeriodicIO.vx = speeds.vxMetersPerSecond;
        mPeriodicIO.vy = speeds.vyMetersPerSecond;
        mPeriodicIO.omega = speeds.omegaRadiansPerSecond;

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

    public void setTeleOpInputs(double throttle, double strafe, double wheel, boolean resetOdometry) {
        setTeleOpInputs(throttle, strafe, wheel, resetOdometry, false);
    }

    public void setTeleOpInputs(double throttle, double strafe, double wheel, boolean resetOdometry, boolean lockTranslation) {
        double xVal = throttle;
        double yVal = -strafe;
        double steerVal = BuzzXboxController.joystickCubicScaledDeadband(
            -wheel, kDriveSteerJoystickDeadbandCutoff, kDriveSteerJoystickWeight
        );
        
        Translation2d translationalInput = new Translation2d(xVal, yVal);

        if(resetOdometry) {
            resetGyro();
            resetOdometry();
        }

        // If no input, keep the swerve modules stopped at their current rotations (rather than snapping to 0 degrees) to avoid skidding
        if(xVal == 0 && yVal == 0 && steerVal == 0 && mPeriodicIO.swerveModuleStates != null) {
            for(int i = 0; i < mPeriodicIO.swerveModuleStates.length; i++) {
                var oldState = mPeriodicIO.swerveModuleStates[i];
                var newState = new SwerveModuleState(0, oldState.angle);
                mPeriodicIO.swerveModuleStates[i] = newState; 
            }

            return;
        }

        if(lockTranslation) {
            var oldDirection = translationalInput.direction();
            var newDirection = Rotation2d.fromDegrees(Math.round(translationalInput.direction().getDegrees() / 45.0) * 45);
            var changeDirection = newDirection.rotateBy(oldDirection.inverse());
            translationalInput = translationalInput.rotateBy(changeDirection);
        }

        // Smooth joystick
        Rotation2d direction = translationalInput.direction();
        double scaledMagnitude = BuzzXboxController.joystickCubicScaledDeadband(translationalInput.norm(), kDriveJoystickDeadbandCutoff, kDriveJoystickWeight);
        translationalInput = new Translation2d(direction.cos() * scaledMagnitude, direction.sin() * scaledMagnitude);

        // Scale magnitude [0, 1] to [0, maxLinearVelocity]
        translationalInput = translationalInput.scale(kDriveMaxLinearVelocity);
        
        double omega = steerVal * kDriveMaxAngularVelocity;

        double filteredYawRate = mYawControlFilter.update(mPeriodicIO.timestamp, mPeriodicIO.yawRate);

        mSteerController.setGoal(omega);
        var correction = mSteerController.calculate(filteredYawRate);
        omega += correction;

        ChassisSpeeds speeds;
        if(mFieldCentric) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translationalInput.x(), translationalInput.y(), omega, getHeadingWPI());
        } else {
            speeds = new ChassisSpeeds(translationalInput.x(), translationalInput.y(), omega);
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
        SmartDashboard.putNumber("v", Math.sqrt(Math.pow(mPeriodicIO.vx, 2) * Math.pow(mPeriodicIO.vy, 2)));
        SmartDashboard.putNumber("omega", mPeriodicIO.omega);

        var pose = mState.poseMeters;
        SmartDashboard.putNumber("lx", Units.metersToInches(pose.getX()));
        SmartDashboard.putNumber("ly", Units.metersToInches(pose.getY()));
        SmartDashboard.putNumber("lv", mState.velocityMetersPerSecond);
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}