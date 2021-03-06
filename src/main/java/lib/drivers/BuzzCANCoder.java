package lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class BuzzCANCoder {
    private CANCoder mCanCoder;
    private CANCoderStickyFaults mCanCoderStickyFaults = new CANCoderStickyFaults();
    
    public BuzzCANCoder(int id, boolean invert) {
        this(id, invert, false);
    }

    public BuzzCANCoder(int id, boolean invert, boolean bootToZero) {
        mCanCoder = new CANCoder(id);
        mCanCoder.configSensorDirection(invert);
        mCanCoder.configSensorInitializationStrategy(bootToZero ? 
            SensorInitializationStrategy.BootToZero : SensorInitializationStrategy.BootToAbsolutePosition);
        mCanCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
        mCanCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 1000);
    }

    public synchronized double getAbsoluteRevs() {
        return mCanCoder.getAbsolutePosition() / 360d;
    }

    public synchronized double getRevs() {
        return mCanCoder.getPosition() / 360d;
    }

    public synchronized double getRevsPerSec() {
        return mCanCoder.getVelocity() / 360d;
    }

    public synchronized boolean getValid() {
        //mCanCoder.getStickyFaults(mCanCoderStickyFaults);
        //mCanCoderStickyFaults.MagnetTooWeak;
        return mCanCoder.getLastError() == ErrorCode.OK;
    }
}