package frc2020.statemachines;

import frc2020.statemachines.SuperstructureStateMachine.WantedAction;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc2020.ShootingLocation.Location;
import frc2020.statemachines.SuperstructureStateMachine.SuperstructureState;

public class SuperstructureStateMachineTest {
    SuperstructureStateMachine stateMachine = new SuperstructureStateMachine();

    @Test
    public void testTransitions() {
        stateMachine.reset();
        
        SuperstructureState state = new SuperstructureState();
        stateMachine.update(0, state, WantedAction.ENABLE_FLYWHEEL, Location.RED, 0);
        stateMachine.update(0, state, WantedAction.SHOOT, Location.RED, 0);
        var c = stateMachine.update(0, state, WantedAction.IDLE, Location.RED, 0);

        assertEquals(58, c.hood, 0.1);
    }
}
