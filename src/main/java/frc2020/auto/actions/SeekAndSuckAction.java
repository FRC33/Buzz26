package frc2020.auto.actions;

import frc2020.RobotState;
import frc2020.subsystems.Drive;
import frc2020.subsystems.Pixy;

public class SeekAndSuckAction implements Action {

    Drive mDrive = Drive.getInstance();
    RobotState mRobotState = RobotState.getInstance();
    Pixy mPixy = Pixy.getInstance();

    public SeekAndSuckAction() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        double wheel = 0;

        if(mPixy.isBallSeen()) {
            wheel = mPixy.getBallAngleX() / 60.0;
        } else {
            var rot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
            var rotAngle = rot.getDegrees();
            if(rotAngle < 0) {
                wheel = -0.8;
            } else {
                wheel = 0.8;
            }
        }

        mDrive.setTeleOpInputs(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {

    }
    
}
