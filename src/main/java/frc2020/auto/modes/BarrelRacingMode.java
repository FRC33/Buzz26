package frc2020.auto.modes;

import frc2020.auto.AutoModeEndedException;
import frc2020.auto.actions.DrivePathAction;
import frc2020.paths.*;

public class BarrelRacingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DrivePathAction(new BarrelRacingPath1(), true));
        //runAction(new DrivePathAction(new BarrelRacingPath2(), true));
        //runAction(new DrivePathAction(new BarrelRacingPath3(), true));
    }
}