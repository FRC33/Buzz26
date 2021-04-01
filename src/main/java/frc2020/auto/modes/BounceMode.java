package frc2020.auto.modes;

import frc2020.auto.AutoModeEndedException;
import frc2020.auto.actions.SwervePathAction;
import frc2020.auto.actions.SwervePathAction.SwervePathActionConstants;
import frc2020.paths.*;

public class BounceMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        var constants = new SwervePathActionConstants();
        constants.kPathXKp = 3.0;
        constants.kPathYKp = 3.0;

        runAction(new SwervePathAction("Bounce1", true, constants));
    }
}