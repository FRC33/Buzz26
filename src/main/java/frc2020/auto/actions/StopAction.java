package frc2020.auto.actions;

import frc2020.RobotState;
import frc2020.subsystems.Drive;
import lib.geometry.Pose2d;

public class StopAction implements Action {

    private Drive mDrive = Drive.getInstance();

    private boolean finished;

    public StopAction() {

    }

    @Override
    public void start() {
        finished = false;
    }

    @Override
    public void update() {
        mDrive.setTeleOpInputs(0, 0, 0);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {}
}