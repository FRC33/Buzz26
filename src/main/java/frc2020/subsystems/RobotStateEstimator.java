package frc2020.subsystems;

import lib.Kinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import frc2020.RobotState;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;
import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Twist2d;

import static frc2020.Constants.*;

//Based on https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/frc2019/subsystems/RobotStateEstimator.java
public class RobotStateEstimator extends Subsystem {
    static RobotStateEstimator mInstance = new RobotStateEstimator();
    private RobotState mRobotState = RobotState.getInstance();
    private Drive mDrive = Drive.getInstance();
    private double prev_timestamp_ = -1.0;
    private Rotation2d prev_heading_ = null;

    private SwerveDriveOdometry mSwerveDriveOdometry;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    private RobotStateEstimator() {
        mSwerveDriveOdometry = new SwerveDriveOdometry(kSwerveKinematics, Rotation2d.fromDegrees(0).toWPI());
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
            prev_timestamp_ = timestamp;
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            if (prev_heading_ == null) {
                prev_heading_ = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
            }

            var modules = mDrive.getSwerveModules();
            var pose = mSwerveDriveOdometry.update(mDrive.getHeading().toWPI(),
                modules[0].getModuleState(),
                modules[1].getModuleState(),
                modules[2].getModuleState(),
                modules[3].getModuleState());

            mRobotState.addFieldToVehicleObservation(timestamp, Pose2d.fromWPI(pose));
        }

        @Override
        public void onStop(double timestamp) {}
    }

    public synchronized void resetOdometry() {
        resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(kInitialHeading)));
    }

    public synchronized void resetOdometry(Pose2d pose) {
        mDrive.resetGyro();

        mSwerveDriveOdometry.resetPosition(
            pose.toWPI(),
            mDrive.getHeading().toWPI()
        );

        mRobotState.reset(Timer.getFPGATimestamp(), pose);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        mRobotState.outputToSmartDashboard();
    }
}