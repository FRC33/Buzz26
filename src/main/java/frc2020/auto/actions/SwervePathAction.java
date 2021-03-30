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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc2020.paths.TrajectoryRegistry;
import frc2020.subsystems.Drive;

public class SwervePathAction implements Action {

    private Drive mDrive = Drive.getInstance();

    private SwerveControllerCommand mSwerveControllerCommand;
    private Trajectory mTrajectory;
    private boolean mResetOdometry;

    private Timer mTimer = new Timer();

    public SwervePathAction(String trajectoryName, boolean resetOdometry) {
        this(trajectoryName, Rotation2d.fromDegrees(0), resetOdometry);
    }

    public SwervePathAction(String trajectoryName, Rotation2d desiredRotation, boolean resetOdometry) {
        this(trajectoryName, () -> desiredRotation, resetOdometry);
    }

    public SwervePathAction(String trajectoryName, Supplier<Rotation2d> desiredRotation, boolean resetOdometry) {
        mTrajectory = TrajectoryRegistry.getInstance().get(trajectoryName);
        
        var xPid = new PIDController(kPathXKp, kPathXKi, kPathXKd);
        var yPid = new PIDController(kPathYKp, kPathYKi, kPathYKd);
        var thetaConstraints = new TrapezoidProfile.Constraints(kPathThetaMaxVelocity, kPathThetaMaxAcceleration);
        var thetaPid = new ProfiledPIDController(
            kPathThetaKp, kPathThetaKi, kPathThetaKd, thetaConstraints
        );
        
        mResetOdometry = resetOdometry;
        mSwerveControllerCommand = new SwerveControllerCommand(
            mTrajectory,
            mDrive::getPoseWPI,
            kSwerveKinematics,
            xPid,
            yPid,
            thetaPid,
            desiredRotation,
            mDrive::setModuleStates
        );
    }

    public Trajectory getTrajectory() {
        return mTrajectory;
    }

    @Override
    public void start() {
        if(mResetOdometry) {
            mDrive.resetOdometry(
                new Pose2d(
                    mTrajectory.getInitialPose().getTranslation(),
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
