package lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import lib.util.Util;

public class BuzzTalonSRX extends TalonSRX {
    
    private BuzzTalonScalars mTalonScalars = new BuzzTalonScalars(1, 1, 4096);
    private double mSaturationVoltage;
    private boolean mVoltageCompEnabled;

    protected ControlMode lastMode = null;
    protected double lastValue = Double.NaN;
    protected ErrorCode lastError = ErrorCode.OK;

    public BuzzTalonSRX(int id) {
        super(id);
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage) {
        if(Util.runWithRetries(() -> super.configVoltageCompSaturation(voltage))) {
            mSaturationVoltage = voltage;
        }
        return getLastError();
    }

    @Override
    public void enableVoltageCompensation(boolean enable) {
        super.enableVoltageCompensation(enable);
        mVoltageCompEnabled = enable;
    }

    public void setDemandVoltage(double demandVoltage) {
        if(mVoltageCompEnabled) {
            set(ControlMode.PercentOutput, demandVoltage / mSaturationVoltage);
        } else {
            set(ControlMode.PercentOutput, demandVoltage / RobotController.getBatteryVoltage());
        }
    }

    public void setBuzzTalonScalars(BuzzTalonScalars talonScalars) {
        mTalonScalars = talonScalars;
    }
    
    public void setBuzzTalonScalars(double reduction, double diameter, int countsPerRev) {
        mTalonScalars = new BuzzTalonScalars(reduction, diameter, countsPerRev);
    }

    public void setRPM(double rpm) {
        setRPS(rpm / 60);
    }

    public void setSurfaceVel(double surfaceVel) {
        setRPS(surfaceVel / (mTalonScalars.getDiameter() * Math.PI));
    }

    public void setSurfacePos(double surfacePos) {
        setRevs(surfacePos / (mTalonScalars.getDiameter() * Math.PI));
    }

    public void setDegrees(double degrees) {
        setRevs(degrees / 360d);
    }

    //region Base set functions
    public void setRPS(double rps) {
        set(ControlMode.Velocity, (rps / 10d) * mTalonScalars.getCountsPerRev() * mTalonScalars.getReduction());
    }

    public void setRevs(double revs) {
        set(ControlMode.Velocity, revs * mTalonScalars.getCountsPerRev() * mTalonScalars.getReduction());
    }
    //endregion

    public double getRPM() {
        return getRPS() * 60;
    }
    
    public double getSurfaceVel() {
        return getRPS() * (mTalonScalars.getDiameter() * Math.PI);
    }

    public double getSurfacePos() {
        return getRevs() * (mTalonScalars.getDiameter() * Math.PI);
    }

    public double getDegrees() {
        return getRevs() * 360d;
    }

    //region Base get functions
    public double getRPS() {
        return ((getSelectedSensorVelocity() / mTalonScalars.getCountsPerRev()) / mTalonScalars.getReduction()) * 10d;
    }

    public double getRevs() {
        return (getSelectedSensorPosition() / mTalonScalars.getCountsPerRev()) / mTalonScalars.getReduction();
    }
    //endregion

    public void configDiffSensors(int D0_id, RemoteSensorSource D0_type, int D1_id, RemoteSensorSource D1_type) {
        configSelectedFeedbackSensor(RemoteFeedbackDevice.SensorDifference);
        configRemoteFeedbackFilter(D0_id, D0_type, 0);
        configRemoteFeedbackFilter(D1_id, D1_type, 1);
        configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0);
        configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1);
    }

    public void configAvgSensors(int D0_id, RemoteSensorSource D0_type, int D1_id, RemoteSensorSource D1_type) {
        configSelectedFeedbackSensor(RemoteFeedbackDevice.SensorSum);
        configRemoteFeedbackFilter(D0_id, D0_type, 0);
        configRemoteFeedbackFilter(D1_id, D1_type, 1);
        configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0);
        configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1);
        configSelectedFeedbackCoefficient(0.5);
    }

    @Override
    public synchronized void set(ControlMode mode, double value) {
        if(mode == ControlMode.PercentOutput) {
            if(value > 1.0) value = 1.0;
            if(value < -1.0) value = -1.0;
        }
        if(mode != lastMode || value != lastValue || lastError != ErrorCode.OK ) {
            super.set(mode, value);
            lastMode = mode;
            lastValue = value;
            lastError = getLastError();
        }
    }
}