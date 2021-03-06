package lib.drivers;

import edu.wpi.first.wpilibj.DigitalInput;
import lib.util.TimeBooleanFilter;
import lib.util.TimeDelayedBoolean;

public class BuzzDigitalInput extends DigitalInput {
    private boolean mInvert;
    private boolean mDebounceEnabled;
    private boolean mLowPassEnabled;
    private double mLowPassTimeS;
    private TimeBooleanFilter mTimeBooleanFilter = new TimeBooleanFilter();

    private boolean lastValue;
    private boolean lastRet;
    
    public BuzzDigitalInput(int id) {
        super(id);
    }

    @Override
    public boolean get() {
        boolean value = super.get() ^ mInvert;
        boolean ret = value;
        if(mDebounceEnabled) {
            ret = (value ^ lastValue) ? lastRet : value;
        } else if(mLowPassEnabled) { 
            ret = mTimeBooleanFilter.update(value, mLowPassTimeS);
        }
        lastValue = value;
        lastRet = ret;
        return ret;
    }

    public void invert(boolean invert) {
        mInvert = invert;
    }

    public void enableDebounce(boolean enable) {
        mDebounceEnabled = enable;
        if(enable) mLowPassEnabled = false;
    }

    public void enableLowPass(boolean enable, double timeS) {
        if(enable) mDebounceEnabled = false;
        mLowPassEnabled = enable;
        mLowPassTimeS = enable ? timeS : 0;
    }
}