package frc2020.subsystems;

import static frc2020.Constants.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import lib.subsystems.Subsystem;
import lib.util.MovingAverage;

public class Pixy extends Subsystem {
    private static Pixy mInstance;

    private DigitalInput mDigitalInput;
    private AnalogInput mAnalogInput;

    public synchronized static Pixy getInstance() {
        if (mInstance == null) {
            mInstance = new Pixy();
        }

        return mInstance;
    }

    private Pixy() {
        mPeriodicIO = new PeriodicIO();

        // Initalize subsystem devices
        mDigitalInput = new DigitalInput(kPixyDigitalInputId);
        mAnalogInput = new AnalogInput(kPixyAnalogInputId);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public boolean ballSeen;
        public double ballAngleX;

        // OUTPUTS
    }

    private MovingAverage mAverage = new MovingAverage(5);
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        mPeriodicIO.ballSeen = mDigitalInput.get();
        if(mPeriodicIO.ballSeen) {
            double ballAngleX = (mAnalogInput.getVoltage() - (3.3 / 2)) * 30;
            mAverage.add(ballAngleX);
            mPeriodicIO.ballAngleX = mAverage.getAverage();
        } else {
            mPeriodicIO.ballAngleX = 0;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {

    }

    public boolean isBallSeen() {
        return mPeriodicIO.ballSeen;
    }

    public double getBallAngleX() {
        return mPeriodicIO.ballAngleX;
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public synchronized void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Pixy Ball Seen", mPeriodicIO.ballSeen);
        SmartDashboard.putNumber("Pixy Ball X", mPeriodicIO.ballAngleX);
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}