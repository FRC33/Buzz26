package lib.util;

public class TorqueLimit {
    private double mMin, mMax;

    public TorqueLimit(double min, double max) {
        mMin = min;
        mMax = max;
    }

    public TorqueLimit limit(TorqueLimit limiter) {
        return new TorqueLimit(
            Math.max(mMin, limiter.getMin()),
            Math.min(mMax, limiter.getMax())
        );
    }

    public double getMin() { return mMin; }
    public double getMax() { return mMax; }
}
