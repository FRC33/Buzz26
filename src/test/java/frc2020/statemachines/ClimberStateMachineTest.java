package frc2020.statemachines;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ClimberStateMachineTest {

    ClimberStateMachine stateMachine = new ClimberStateMachine();

    @Test
    public void testTransitions() {
        stateMachine.reset();
        
        stateMachine.update(0, ClimberStateMachine.WantedAction.MAST_UP);
        assertEquals(ClimberStateMachine.SystemState.MAST_RUNNING_UP, stateMachine.getSystemState());
        stateMachine.update(0, ClimberStateMachine.WantedAction.STOP);
        assertEquals(ClimberStateMachine.SystemState.MAST_RAN, stateMachine.getSystemState());

        stateMachine.update(0, ClimberStateMachine.WantedAction.WINCH_IN);
        assertEquals(ClimberStateMachine.SystemState.WINCH_RUNNING_IN, stateMachine.getSystemState());
        stateMachine.update(0, ClimberStateMachine.WantedAction.STOP);
        assertEquals(ClimberStateMachine.SystemState.WINCHED_IN, stateMachine.getSystemState());
    }
}