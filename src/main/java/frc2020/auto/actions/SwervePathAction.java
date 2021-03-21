package frc2020.auto.actions;

import static frc2020.Constants.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc2020.subsystems.Drive;

public class SwervePathAction implements Action {

    private Drive mDrive = Drive.getInstance();

    private SwerveControllerCommand mSwerveControllerCommand;
    private Trajectory mTrajectory;

    public SwervePathAction(Trajectory trajectory) {
        this(trajectory, Rotation2d.fromDegrees(0));
    }

    public SwervePathAction(Trajectory trajectory, Rotation2d desiredRotation) {
        this(trajectory, () -> desiredRotation);
    }

    public SwervePathAction(Trajectory trajectory, Supplier<Rotation2d> desiredRotation) {
        var xPid = new PIDController(0, 0, 0);
        var yPid = new PIDController(0, 0, 0);
        var thetaConstraints = new TrapezoidProfile.Constraints(0, 0);
        var thetaPid = new ProfiledPIDController(0, 0, 0, thetaConstraints);
        
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
