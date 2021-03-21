package frc2020.auto.modes;

import frc2020.auto.AutoModeEndedException;
import frc2020.auto.actions.SwervePathAction;
import frc2020.paths.*;

public class SlalomMode extends AutoModeBase {

    private SwervePathAction action;

    public SlalomMode() {
        // This is in the constructor so that the path is loaded earlier
        action = new SwervePathAction("Slalom", true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(action);
    }
}