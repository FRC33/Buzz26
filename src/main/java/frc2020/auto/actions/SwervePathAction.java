package frc2020.auto.actions;

import static frc2020.Constants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc2020.RobotState;
import frc2020.paths.TrajectoryRegistry;
import frc2020.subsystems.Drive;
import frc2020.subsystems.RobotStateEstimator;
import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;

public class SwervePathAction implements Action {

    private Drive mDrive = Drive.getInstance();
    private RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private SwerveControllerCommand mSwerveControllerCommand;
    private Trajectory mTrajectory;
    private boolean mResetOdometry;
    private SwervePathActionConstants mConstants;

    private Timer mTimer = new Timer();

    public static class SwervePathActionConstants {
        public double kPathXKp = 0.5;
        public double kPathXKi = 0;
        public double kPathXKd = 0;
    
        public double kPathYKp = 0.5;
        public double kPathYKi = 0;
        public double kPathYKd = 0;
    
        public double kPathThetaKp = 0;
        public double kPathThetaKi = 0;
        public double kPathThetaKd = 0;
        public double kPathThetaMaxVelocity = 50 / 16;
        public double kPathThetaMaxAcceleration = 170 / 16;
    }

    public SwervePathAction(String trajectoryName, boolean resetOdometry) {
        this(trajectoryName, resetOdometry, new SwervePathActionConstants());
    }

    public SwervePathAction(String trajectoryName, Rotation2d desiredRotation, boolean resetOdometry) {
        this(trajectoryName, desiredRotation, resetOdometry, new SwervePathActionConstants());
    }

    public SwervePathAction(String trajectoryName, Supplier<Rotation2d> desiredRotation, boolean resetOdometry) {
        this(trajectoryName, desiredRotation, resetOdometry, new SwervePathActionConstants());
    }
    
    public SwervePathAction(String trajectoryName, boolean resetOdometry, SwervePathActionConstants constants) {
        this(trajectoryName, Rotation2d.fromDegrees(0), resetOdometry, constants);
    }

    public SwervePathAction(String trajectoryName, Rotation2d desiredRotation, boolean resetOdometry, SwervePathActionConstants constants) {
        this(trajectoryName, () -> desiredRotation, resetOdometry, constants);
    }

    public SwervePathAction(String trajectoryName, Supplier<Rotation2d> desiredRotation, boolean resetOdometry, SwervePathActionConstants constants) {
        mTrajectory = TrajectoryRegistry.getInstance().get(trajectoryName);
        mConstants = constants;
        
        var xPid = new PIDController(mConstants.kPathXKp, mConstants.kPathXKi, mConstants.kPathXKd);
        var yPid = new PIDController(mConstants.kPathYKp, mConstants.kPathYKi, mConstants.kPathYKd);
        var thetaConstraints = new TrapezoidProfile.Constraints(mConstants.kPathThetaMaxVelocity, mConstants.kPathThetaMaxAcceleration);
        var thetaPid = new ProfiledPIDController(
            mConstants.kPathThetaKp, mConstants.kPathThetaKi, mConstants.kPathThetaKd, thetaConstraints
        );
        
        mResetOdometry = resetOdometry;
        mSwerveControllerCommand = new SwerveControllerCommand(
            mTrajectory,
            () -> mRobotState.getLatestFieldToVehicle().getValue().toWPI(),
            kSwerveKinematics,
            xPid,
            yPid,
            thetaPid,
            () -> desiredRotation.get().toWPI(),
            mDrive::setModuleStates
        );
    }

    public Trajectory getTrajectory() {
        return mTrajectory;
    }

    @Override
    public void start() {
        if(mResetOdometry) {
            mRobotStateEstimator.resetOdometry(
                new Pose2d(
                    Translation2d.fromWPI(mTrajectory.getInitialPose().getTranslation()),
                    Rotation2d.fromDegrees(0)
                )
            );
        }

        mSwerveControllerCommand.initialize();

        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void update() {
        mSwerveControllerCommand.execute();

        // Set trajectory state in drive to display on dashboard
        mDrive.setTrajectoryState(mTrajectory.sample(mTimer.get()));
    }

    @Override
    public boolean isFinished() {
        return mSwerveControllerCommand.isFinished();
    }

    @Override
    public void done() {
        mSwerveControllerCommand.end(false);
        mTimer.stop();
    }
}
