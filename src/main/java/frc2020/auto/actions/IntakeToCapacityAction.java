package frc2020.auto.actions;

import frc2020.statemachines.SuperstructureStateMachine.SystemState;
import frc2020.subsystems.Superstructure;

public class IntakeToCapacityAction implements Action {

    Superstructure mSuperstructure = Superstructure.getInstance();

    @Override
    public void start() {
        mSuperstructure.setWantIntakeOn();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return mSuperstructure.getSystemState() == SystemState.INTAKE_FINISH;
    }

    @Override
    public void done() {
        mSuperstructure.setWantIdle();
    }
}
