package frc2020.auto.modes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.auto.AutoModeEndedException;
import frc2020.auto.actions.LambdaAction;
import frc2020.auto.actions.ParallelAction;
import frc2020.auto.actions.SeekAndSuckAction;
import frc2020.auto.actions.SelectAction;
import frc2020.auto.actions.SeriesAction;
import frc2020.auto.actions.SwervePathAction;
import frc2020.auto.actions.HoodAngleAction;
import frc2020.auto.actions.IntakeToCapacityAction;
import frc2020.paths.*;
import frc2020.subsystems.Drive;
import frc2020.subsystems.Inventory;
import frc2020.subsystems.Pixy;
import frc2020.subsystems.Superstructure;

public class GalacticSearchMode extends AutoModeBase {
    Pixy mPixy = Pixy.getInstance();
    Drive mDrive = Drive.getInstance();
    Superstructure mSuperstructure = Superstructure.getInstance();
    Inventory mInventory = Inventory.getInstance();

    Timer mTimer = new Timer();
    
    @Override
    protected void routine() throws AutoModeEndedException {
        mTimer.reset();
        mTimer.start();

        runAction(
            new ParallelAction(
                new IntakeToCapacityAction(),
                new SwervePathAction("RedA", this::getTargetHeading, true)
            )
        );


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

    private Rotation2d getTargetHeading() {
        var ballCount = mInventory.getBallCount();
        
        if(mDrive.getPoseWPI().getX() >= 3.81) {
            return Rotation2d.fromDegrees(90);
        } else {
            return Rotation2d.fromDegrees(0);
        }
    }

    private boolean isBallSeen() {
        return mPixy.isBallSeen();
    }

    private boolean isBallToRight() {
        return mPixy.getBallAngleX() >= 0;
    }
}