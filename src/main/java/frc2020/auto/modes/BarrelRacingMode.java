package frc2020.auto.modes;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.auto.AutoModeEndedException;
import frc2020.paths.*;
import frc2020.subsystems.Drive;

public class BarrelRacingMode extends AutoModeBase {

    Drive mDrive = Drive.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {
        mDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }
}