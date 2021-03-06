package lib.util;

public class BooleanFilter {
    private boolean mLastRet;
    private final double mDelay;

    private DelayedBoolean mDelayedBoolean1;
    private DelayedBoolean mDelayedBoolean2;

    public BooleanFilter(double timestamp, double delay) {
        mLastRet = false;
        mDelay = delay;

        mDelayedBoolean1 = new DelayedBoolean(timestamp, mDelay);
        mDelayedBoolean2 = new DelayedBoolean(timestamp, mDelay);
    }

    public boolean update(double timestamp, boolean value) {
        boolean delayedBoolean1Value = 
            mDelayedBoolean1.update(timestamp, value & !mLastRet);
        boolean delayedBoolean2Value = 
            mDelayedBoolean1.update(timestamp, value & !mLastRet);

        boolean ret = delayedBoolean1Value ^ delayedBoolean2Value ^ mLastRet;
        
        mLastRet = ret;
        return ret;
    }
}
