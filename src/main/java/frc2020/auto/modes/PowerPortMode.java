package frc2020.auto.modes;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.auto.AutoModeEndedException;
import frc2020.auto.actions.LambdaAction;
import frc2020.auto.actions.SwervePathAction;
import frc2020.auto.actions.WaitAction;
import frc2020.auto.actions.SwervePathAction.SwervePathActionConstants;
import frc2020.auto.actions.*;
import frc2020.paths.*;
import frc2020.subsystems.Drive;

public class PowerPortMode extends AutoModeBase {

    Drive mDrive = Drive.getInstance();

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

        for(int i = 0; i < 5; i++) {
            runAction(new SwervePathAction("PowerPort", Rotation2d.fromDegrees(180), i == 0, constants));
            runAction(new LambdaAction(() -> mDrive.centerWheels()));
            runAction(new WaitAction(1.0)); // Intake
            runAction(new SwervePathAction("PowerPortBack", Rotation2d.fromDegrees(180), false, constants));
            runAction(new AimAction());
            runAction(new LambdaAction(() -> mDrive.centerWheels()));
            runAction(new WaitAction(0.3));
        }
    }
}