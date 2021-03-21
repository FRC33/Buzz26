package frc2020.auto.actions;

import static frc2020.Constants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc2020.subsystems.Drive;

public class SwervePathAction implements Action {

    private Drive mDrive = Drive.getInstance();

    private SwerveControllerCommand mSwerveControllerCommand;
    private Trajectory mTrajectory;

    public SwervePathAction(String trajectoryName) {
        this(loadTrajectory(trajectoryName));
    }

    public SwervePathAction(String trajectoryName, Rotation2d desiredRotation) {
        this(loadTrajectory(trajectoryName), desiredRotation);
    }

    public SwervePathAction(String trajectoryName, Supplier<Rotation2d> desiredRotation) {
        this(loadTrajectory(trajectoryName), desiredRotation);
    }

    public SwervePathAction(Trajectory trajectory) {
        this(trajectory, Rotation2d.fromDegrees(0));
    }

    public SwervePathAction(Trajectory trajectory, Rotation2d desiredRotation) {
        this(trajectory, () -> desiredRotation);
    }

    public SwervePathAction(Trajectory trajectory, Supplier<Rotation2d> desiredRotation) {
        var xPid = new PIDController(kPathXKp, kPathXKi, kPathXKd);
        var yPid = new PIDController(kPathYKp, kPathYKi, kPathYKd);
        var thetaConstraints = new TrapezoidProfile.Constraints(kPathThetaMaxVelocity, kPathThetaMaxAcceleration);
        var thetaPid = new ProfiledPIDController(
            kPathThetaKp, kPathThetaKi, kPathThetaKd, thetaConstraints
        );
        
        mTrajectory = trajectory;
        mSwerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            mDrive::getPoseWPI,
            kSwerveKinematics,
            xPid,
            yPid,
            thetaPid,
            desiredRotation,
            mDrive::setModuleStates
        );
    }

    public static Trajectory loadTrajectory(String name) {
        String trajectoryJSON = "paths/" + name + ".wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return trajectory;
    }

    @Override
    public void start() {
        mSwerveControllerCommand.initialize();
    }

    @Override
    public void update() {
        mSwerveControllerCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return mSwerveControllerCommand.isFinished();
    }

    @Override
    public void done() {
        mSwerveControllerCommand.end(false);
    }
}
