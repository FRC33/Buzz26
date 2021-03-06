package frc2020.states;

public class LEDState {
    private Mode mMode = Mode.IDLE;
    private int mCount = 0;
    private double mAutoAimError = 0;
    private boolean mPartyMode = false;

    public LEDState() {

    }

    public void set(Mode mode, int count, double autoAimError, boolean partyMode) {
        mMode = mode;
        mCount = count;
        mAutoAimError = autoAimError;
        mPartyMode = partyMode;
    }

    public Mode getMode() {
        return mMode;
    }

    public int getCount() {
        return mCount;
    }

    public double getAutoAimError() {
        return mAutoAimError;
    }

    public boolean getPartyMode() {
        return mPartyMode;
    }

    public static enum Mode {
        IDLE,
        INTAKE,
        BUZZ_STYLE
    }
}