package frc2020.auto.modes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.auto.AutoModeEndedException;
import frc2020.auto.actions.LambdaAction;
import frc2020.auto.actions.ParallelAction;
import frc2020.auto.actions.SeekAndSuckAction;
import frc2020.auto.actions.SelectAction;
import frc2020.auto.actions.SeriesAction;
import frc2020.auto.actions.HoodAngleAction;
import frc2020.auto.actions.IntakeToCapacityAction;
import frc2020.paths.*;
import frc2020.subsystems.Pixy;
import frc2020.subsystems.Superstructure;

public class GalacticSearchMode extends AutoModeBase {
    Pixy mPixy = Pixy.getInstance();
    Superstructure mSuperstructure = Superstructure.getInstance();
    
    @Override
    protected void routine() throws AutoModeEndedException {
        /*

        runAction(new HoodAngleAction(58, 0.5));
        runAction(new SelectAction(this::isBallSeen, 
            // If ball seen, run red paths
            // The following series action only contains one action,
            //  it is just there so that it looks similar to the blue case
            new SeriesAction(
                new ParallelAction(
                    new IntakeToCapacityAction(),
                    new SelectAction(this::isBallToRight, 
                        new DrivePathAction(new GalacticSearchPathRedA()),
                        new DrivePathAction(new GalacticSearchPathRedB())
                    ),
                    new LambdaAction(() -> {
                        // LED and dashboard output could go here
                    })
                )
            ),
            // Otherwise, run blue paths
            new SeriesAction(
                new HoodAngleAction(70, 0.5),
                new ParallelAction(
                    new IntakeToCapacityAction(),
                    new SelectAction(this::isBallToRight, 
                        new DrivePathAction(new GalacticSearchPathBlueA()),
                        new DrivePathAction(new GalacticSearchPathBlueB())
                    ),
                    new LambdaAction(() -> {
                        // LED and dashboard output could go here
                    })
                )
            )
        ));

        */
    }

    private boolean isBallSeen() {
        return mPixy.isBallSeen();
    }

    private boolean isBallToRight() {
        return mPixy.getBallAngleX() >= 0;
    }
}