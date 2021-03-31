package frc2020.auto.modes;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.auto.AutoModeEndedException;
import frc2020.auto.actions.SwervePathAction;
import frc2020.paths.*;
import frc2020.subsystems.Drive;

public class BarrelRacingMode extends AutoModeBase {

    Drive mDrive = Drive.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SwervePathAction("Barrel", true));
    }
}