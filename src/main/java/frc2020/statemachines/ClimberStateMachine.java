package frc2020.statemachines;

public class ClimberStateMachine {
    private final double kMastIdleVoltage = -0.5;
    //private final double kMastReturnVoltage = -3; //Not used
    private final double kMastUpVotlage = 13;
    private final double kMastDownVotlage = -13;

    private final double kWinchOutVoltage = 13;
    private final double kWinchInVoltage = -13;

    private SystemState mSystemState = SystemState.IDLE;
    private ClimberState mDesiredState = new ClimberState();

    public ClimberStateMachine() {
        
    }

    public enum SystemState {
        IDLE, MAST_RUNNING_UP, MAST_RUNNING_DOWN, WINCH_RUNNING_OUT, WINCH_RUNNING_IN, MAST_RAN, WINCHED_IN
    }

    public enum WantedAction {
        STOP, MAST_UP, MAST_DOWN, WINCH_OUT, WINCH_IN
    }

    public static class ClimberState {
        public double mastVoltage;
        public double winchVoltage;
        public boolean partyMode;
    }

    public synchronized void reset() {
        mSystemState = SystemState.IDLE;
    }
    
    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    public synchronized ClimberState update(double timestamp, WantedAction wantedAction) {
        SystemState newState = mSystemState;

        switch (mSystemState) {
            case IDLE:
                newState = handleIdleTransitions(wantedAction);
                break;
            case MAST_RUNNING_UP:
                newState = handleRunningTransitions(wantedAction);
                break;
            case MAST_RUNNING_DOWN:
                newState = handleRunningTransitions(wantedAction);
                break;
            case WINCH_RUNNING_OUT:
                newState = handleRunningTransitions(wantedAction);
                break;
            case WINCH_RUNNING_IN:
                newState = handleWinchRunningInTransitions(wantedAction);
                break;
            case MAST_RAN:
                newState = handleRanTransitions(wantedAction);
                break;
            case WINCHED_IN:
                newState = handleWinchedInTransitions(wantedAction);
                break;
        }

        mSystemState = newState;

        mDesiredState.mastVoltage = 0;
        mDesiredState.winchVoltage = 0;
        mDesiredState.partyMode = false;
        switch (mSystemState) {
            case IDLE:
                getIdleDesiredState();
                break;
            case MAST_RUNNING_UP:
                getMastRunningUpDesiredState();
                break;
            case MAST_RUNNING_DOWN:
                getMastRunningDownDesiredState();
                break;
            case WINCH_RUNNING_OUT:
                getWinchRunningOutDesiredState();
                break;
            case WINCH_RUNNING_IN:
                getWinchRunningInDesiredState();
                break;
            case MAST_RAN:
                getMastRanDesiredState();
                break;
            case WINCHED_IN:
                getWinchedInDesiredState();
                break;
        }

        return mDesiredState;
    }

    private SystemState handleIdleTransitions(WantedAction wantedAction) {
        if(wantedAction == WantedAction.STOP) {
            return SystemState.IDLE;
        }

        return defaultTransitions(wantedAction);
    }

    private SystemState handleRunningTransitions(WantedAction wantedAction) {
        if(wantedAction == WantedAction.STOP) {
            return SystemState.MAST_RAN;
        }

        return defaultTransitions(wantedAction);
    }

    private SystemState handleWinchRunningInTransitions(WantedAction wantedAction) {
        if(wantedAction == WantedAction.STOP) {
            return SystemState.WINCHED_IN;
        }

        return defaultTransitions(wantedAction);
    }

    private SystemState handleRanTransitions(WantedAction wantedAction) {
        return defaultTransitions(wantedAction);
    }

    private SystemState handleWinchedInTransitions(WantedAction wantedAction) {
        if(wantedAction == WantedAction.STOP) {
            return SystemState.WINCHED_IN;
        }

        return defaultTransitions(wantedAction);
    }

    private SystemState defaultTransitions(WantedAction wantedAction) {
        switch (wantedAction) {
            case STOP:
                return SystemState.MAST_RAN;
            case MAST_UP:
                return SystemState.MAST_RUNNING_UP;
            case MAST_DOWN:
                return SystemState.MAST_RUNNING_DOWN;
            case WINCH_OUT:
                return SystemState.WINCH_RUNNING_OUT;
            case WINCH_IN:
                return SystemState.WINCH_RUNNING_IN;
            default:
                return SystemState.MAST_RAN;
        }
    }

    private void getIdleDesiredState() {
        mDesiredState.mastVoltage = kMastIdleVoltage;
    }

    private void getMastRunningUpDesiredState() {
        mDesiredState.mastVoltage = kMastUpVotlage;
    }

    private void getMastRunningDownDesiredState() {
        mDesiredState.mastVoltage = kMastDownVotlage;
    }

    private void getWinchRunningOutDesiredState() {
        mDesiredState.winchVoltage = kWinchOutVoltage;
    }

    private void getWinchRunningInDesiredState() {
        mDesiredState.winchVoltage = kWinchInVoltage;
    }

    private void getMastRanDesiredState() {
        //Defaults
    }

    private void getWinchedInDesiredState() {
        mDesiredState.partyMode = true;
    }
}