package lib.util;

/**
 * Based off of VI from Buzz 25 LabVIEW code
 */
public class LeadLagFilter {
    private final double mLead;
    private final double mLag;

    private double mLastInput = 0;
    private double mLastOutput = 0;
    private double mLastTimestamp;

    public LeadLagFilter(double timestamp, double lead, double lag) {
        mLastTimestamp = timestamp;
        mLead = lead;
        mLag = lag;
    }

    public double update(double timestamp, double input) {
        double dt = timestamp - mLastTimestamp;
        double c1 = mLag / (mLag + dt);
        double c2 = (mLead + dt) / (mLag + dt);
        double c3 = -mLead / (mLag + dt);
        double output = (c1 * mLastOutput) + (c2 * input) + (c3 * mLastInput);

        mLastInput = input;
        mLastOutput = output;
        mLastTimestamp = timestamp;

        return output;
    }

    public void reset(double timestamp) {
        mLastTimestamp = timestamp;
        mLastInput = 0;
        mLastOutput = 0;
    }
}
