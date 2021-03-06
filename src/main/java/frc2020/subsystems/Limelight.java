package frc2020.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import lib.subsystems.Subsystem;

public class Limelight extends Subsystem {
    private static Limelight mInstance;

    private NetworkTableEntry txEntry;
    private NetworkTableEntry tyEntry;
    private NetworkTableEntry taEntry;
    private NetworkTableEntry tsEntry;
    private NetworkTableEntry tlEntry;
    private NetworkTableEntry tvEntry;
    private NetworkTableEntry getpipeEntry;

    private NetworkTableEntry ledModeEntry;
    private NetworkTableEntry pipelineEntry;

    private boolean outputsHaveChanged;

    public synchronized static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }

        return mInstance;
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double tx;
        public double ty;
        public double ta;
        public double ts;
        public double tl;
        public double tv;
        public int getpipe;

        // OUTPUTS
        public int pipeline;
        public int ledMode;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        mPeriodicIO.tx = txEntry.getDouble(0);
        mPeriodicIO.ty = tyEntry.getDouble(0);
        mPeriodicIO.ta = taEntry.getDouble(0);
        mPeriodicIO.ts = tsEntry.getDouble(0);
        mPeriodicIO.tl = tlEntry.getDouble(0);
        mPeriodicIO.tv = tvEntry.getDouble(0);
        mPeriodicIO.getpipe = (int) getpipeEntry.getDouble(0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output
        if(outputsHaveChanged) {
            ledModeEntry.setDouble(mPeriodicIO.ledMode);
            pipelineEntry.setDouble(mPeriodicIO.pipeline);
            outputsHaveChanged = false;
        }
    }

    private Limelight() {
        mPeriodicIO = new PeriodicIO();

        //Initalize network table
        var table = NetworkTableInstance.getDefault().getTable("limelight");

        txEntry = table.getEntry("tx");
        tyEntry = table.getEntry("ty");
        taEntry = table.getEntry("ta");
        tsEntry = table.getEntry("ts");
        tlEntry = table.getEntry("tl");
        tvEntry = table.getEntry("tv");
        getpipeEntry = table.getEntry("getpipe");

        ledModeEntry = table.getEntry("ledMode");
        ledModeEntry = table.getEntry("ledMode");
    }

    public double getXAngle() {
        return mPeriodicIO.tx;
    }

    public double getYAngle() {
        return mPeriodicIO.ty;
    }

    public double getPercentArea() {
        return mPeriodicIO.ta;
    }

    public double getSkew() {
        return mPeriodicIO.ts;
    }

    public double getLatency() {
        return mPeriodicIO.tl + 11;
    }

    public boolean getValid() {
        return mPeriodicIO.tv == 1;
    }

    public int getActivePipeline() {
        return mPeriodicIO.getpipe;
    }

    public void setLedMode(LedMode ledMode) {
        if(mPeriodicIO.ledMode != ledMode.getIndex()) {
            mPeriodicIO.ledMode = ledMode.getIndex();
            outputsHaveChanged = true;
        }
    }

    public void setDesiredPipeline(int pipeline) {
        if(mPeriodicIO.pipeline != pipeline) {
            mPeriodicIO.pipeline = pipeline;
            outputsHaveChanged = true;
        }
    }

    public enum LedMode {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        private int mIndex;

        public int getIndex() {
            return mIndex;
        }

        private LedMode(int index) {
            mIndex = index;
        }
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
        
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}