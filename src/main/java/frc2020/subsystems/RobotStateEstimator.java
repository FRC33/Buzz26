package frc2020.subsystems;

import lib.Kinematics;
import frc2020.RobotState;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;
import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Twist2d;

//Based on https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/frc2019/subsystems/RobotStateEstimator.java
public class RobotStateEstimator extends Subsystem {
    static RobotStateEstimator mInstance = new RobotStateEstimator();
    private RobotState mRobotState = RobotState.getInstance();
    private Drive mDrive = Drive.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double prev_timestamp_ = -1.0;
    private Rotation2d prev_heading_ = null;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    private RobotStateEstimator() {}

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
            left_encoder_prev_distance_ = mDrive.getLeftPosition(); //getLeftEncoderDistance
            right_encoder_prev_distance_ = mDrive.getRightPosition(); //getRightEncoderDistance
            prev_timestamp_ = timestamp;
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            if (prev_heading_ == null) {
                prev_heading_ = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
            }
            final double dt = timestamp - prev_timestamp_;
            final double left_distance = mDrive.getLeftPosition(); //getLeftEncoderDistance
            final double right_distance = mDrive.getRightPosition(); //getRightEncoderDistance
            final double delta_left = left_distance - left_encoder_prev_distance_;
            final double delta_right = right_distance - right_encoder_prev_distance_;
            final Rotation2d gyro_angle = mDrive.getHeading();
            Twist2d odometry_twist;
            synchronized (mRobotState) {
                final Pose2d last_measurement = mRobotState.getLatestFieldToVehicle().getValue();
                odometry_twist = Kinematics.forwardKinematics(last_measurement.getRotation(), delta_left,
                        delta_right, gyro_angle);
            }
            final Twist2d measured_velocity = Kinematics.forwardKinematics(
                    delta_left, delta_right, prev_heading_.inverse().rotateBy(gyro_angle).getRadians()).scaled(1.0 / dt);
            final Twist2d predicted_velocity = Kinematics.forwardKinematics(mDrive.getLeftVelocity(), //getLeftLinearVelocity
                    mDrive.getRightVelocity()).scaled(dt);
            mRobotState.addVehicleToTurretObservation(timestamp,
                    Rotation2d.fromDegrees(Turret.getInstance().getAngle()));
            mRobotState.addObservations(timestamp, odometry_twist, measured_velocity, predicted_velocity);
            left_encoder_prev_distance_ = left_distance;
            right_encoder_prev_distance_ = right_distance;
            prev_heading_ = gyro_angle;
            prev_timestamp_ = timestamp;
        }

        @Override
        public void onStop(double timestamp) {}
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