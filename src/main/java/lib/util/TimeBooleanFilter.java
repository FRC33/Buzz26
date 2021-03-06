package lib.util;

public class TimeBooleanFilter {
    private boolean mLastRet;
    
    private TimeDelayedBoolean mDelayedBoolean1;
    private TimeDelayedBoolean mDelayedBoolean2;

    public TimeBooleanFilter() {
        mLastRet = false;

        mDelayedBoolean1 = new TimeDelayedBoolean();
        mDelayedBoolean2 = new TimeDelayedBoolean();
    }

    public boolean update(boolean value, double delay) {
        boolean delayedBoolean1Value = 
            mDelayedBoolean1.update(value & !mLastRet, delay);
        boolean delayedBoolean2Value = 
            mDelayedBoolean2.update(!value & mLastRet, delay);

        boolean ret = delayedBoolean1Value ^ delayedBoolean2Value ^ mLastRet;
        
        mLastRet = ret;
        return ret;
    }
}
