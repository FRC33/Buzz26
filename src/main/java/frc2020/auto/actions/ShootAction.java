package frc2020.auto.actions;

import frc2020.subsystems.Drive;
import frc2020.subsystems.Superstructure;

public class ShootAction implements Action {

    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();

    @Override
    public void start() {
        mSuperstructure.setWantShoot();
    }

    @Override
    public void update() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {
        mSuperstructure.setWantIdle();
    }
    
}
