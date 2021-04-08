package frc2020.auto.modes;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.auto.AutoModeEndedException;
import frc2020.auto.actions.SwervePathAction.SwervePathActionConstants;
import frc2020.hmi.HMI;
import frc2020.auto.actions.*;
import frc2020.paths.*;
import frc2020.subsystems.Drive;

public class PowerPortMode extends AutoModeBase {

    Drive mDrive = Drive.getInstance();
    HMI mHMI = HMI.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {
        var constants = new SwervePathActionConstants();
        constants.kPathXKp = 5.0;
        constants.kPathXKd = 0;
        constants.kPathYKp = 5.0;
        constants.kPathYKd = 0;
        constants.kPathThetaKp = 0;
        constants.kPathThetaMaxVelocity = 50.0 / 16.0;
        constants.kPathThetaMaxAcceleration = 170.0 / 16.0;

        // Shoot preload
        runAction(new HangingAction(new ShootAction(), mHMI.getDriver()::getAButton));
        runAction(new LambdaAction(() -> mDrive.centerWheels()));
        runAction(new WaitAction(0.2));

        for(int i = 0; i < 6; i++) {
            // Drive to reintroduction zone
            runAction(new SwervePathAction("PowerPort", this::getTargetAngle, i == 0, constants));
            runAction(new LambdaAction(mDrive::centerWheels));
 
            runAction(new RaceAction(
                // Start intaking
                new IntakeToCapacityAction(),
                // Start driving to shooting zone once B is pressed
                new SeriesAction(
                    new WaitLambdaAction(mHMI.getDriver()::getBButton),
                    new SwervePathAction("PowerPortBack", this::getTargetAngle, false, constants)
                )
            ));
            
            // Aim until 0.3 degrees from target
            runAction(new AimAction(0.3));

            // Shoot until A button is pressed
            runAction(new LambdaAction(mDrive::lockWheels));
            runAction(new HangingAction(new ShootAction(), mHMI.getDriver()::getAButton));

            // Prepare to drive to reintroduction zone
            runAction(new LambdaAction(mDrive::centerWheels));
            runAction(new WaitAction(0.2));
        }
    }

    // Ensures the target angle (180 or -180 degrees) is the same sign as the current angle
    // so that the control loop always outputs in the correct direction
    public Rotation2d getTargetAngle() {
        return new Rotation2d(Math.signum(mDrive.getPoseWPI().getRotation().getRadians()) * Math.PI);
    }
}