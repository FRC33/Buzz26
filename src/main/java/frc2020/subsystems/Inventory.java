package frc2020.subsystems;

import static frc2020.Constants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.drivers.BuzzDigitalInput;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;
import lib.util.LatchedBoolean;

public class Inventory extends Subsystem {
    private static Inventory mInstance;

    private BuzzDigitalInput mBallSensors[] = new BuzzDigitalInput[kBallSensorIds.length];

    private LatchedBoolean mBallEnteredLatch = new LatchedBoolean();

    private int mVisibleBallCount;
    private int mBallCount;

    // Set from superstructure
    private boolean mIntaking;

    public synchronized static Inventory getInstance() {
        if (mInstance == null) {
            mInstance = new Inventory();
        }

        return mInstance;
    }

    private Inventory() {
        mPeriodicIO = new PeriodicIO();

        for(int i = 0; i < kBallSensorIds.length; i++) {
            mBallSensors[i] = new BuzzDigitalInput(kBallSensorIds[i]);
        }
        
        mBallSensors[0].invert(false);
        mBallSensors[1].invert(false);
        mBallSensors[2].invert(false);

        mBallSensors[0].enableLowPass(true, 0.04);
        mBallSensors[1].enableLowPass(true, 0.03);
        mBallSensors[2].enableLowPass(true, 0.02);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public boolean[] sensorValues = new boolean[kBallSensorIds.length];
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        for(int i = 0; i < mBallSensors.length; i++) {
            mPeriodicIO.sensorValues[i] = mBallSensors[i].get();
        }
    }

    @Override
    public void registerEnabledLoops(final ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(final double timestamp) {
                synchronized (Inventory.this) {
                    /*
                    if(getVisbileBalls() == 0) {
                        mBallCount = 0;
                    } else {
                        mBallCount = 3;
                    }
                    */
                    mBallCount = 0;
                }
            }

            @Override
            public void onLoop(final double timestamp) {
                synchronized (Inventory.this) {
                    // Visible ball count
                    int newBallCount = 0;
                    for(int i = 0; i < mPeriodicIO.sensorValues.length; i++) {
                        newBallCount += mPeriodicIO.sensorValues[i] ? 1 : 0;
                    }
                    mVisibleBallCount = newBallCount;

                    // Ball entered
                    var ballEnteredLatchVal = mBallEnteredLatch.update(mPeriodicIO.sensorValues[0]);
                    if(mIntaking && ballEnteredLatchVal && mBallCount < 3) {
                        mBallCount++;
                    }
                }
            }

            @Override
            public void onStop(final double timestamp) {
                stop();
            }
        });
    }

    public synchronized boolean[] getSensorValues() {
        return mPeriodicIO.sensorValues;
    }

    public synchronized int getVisbileBalls() {
        return mVisibleBallCount;
    }

    public synchronized int getBallCount() {
        return mBallCount;
    }

    public void setIntaking(boolean intaking) {
        this.mIntaking = intaking; 
    }

    public void resetBallCount() {
        this.mBallCount = 0;
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
        SmartDashboard.putNumber("Visible Ball Count", getVisbileBalls());
        SmartDashboard.putNumber("Tracked Ball Count", getBallCount());
        SmartDashboard.putBooleanArray("Ball Sensor Values", mPeriodicIO.sensorValues);
        SmartDashboard.putBoolean("Ball Sensor ID 1", mPeriodicIO.sensorValues[0]);
    }
}