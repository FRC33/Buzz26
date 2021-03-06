package frc2020.auto.modes;

import frc2020.auto.AutoModeEndedException;
import frc2020.auto.actions.DrivePathAction;
import frc2020.paths.*;

public class BounceMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        //TODO
        runAction(new DrivePathAction(new BouncePath1()));
        runAction(new DrivePathAction(new BouncePath2()));
        runAction(new DrivePathAction(new BouncePath3()));
        runAction(new DrivePathAction(new BouncePath4()));
    }
}