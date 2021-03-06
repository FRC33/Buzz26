package frc2020.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.drivers.BuzzCANCoder;
import lib.drivers.BuzzTalonFX;
import lib.drivers.TalonFXFactory;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;
import lib.util.MovingAverage;
import lib.util.SynchronousPIDF;

import static frc2020.Constants.*;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Turret extends Subsystem {
    private static Turret mInstance;

    private BuzzTalonFX mAzimuth;
    private BuzzCANCoder mEncoderA;
    private BuzzCANCoder mEncoderB;

    private SynchronousPIDF pidf = new SynchronousPIDF(0.25, 0.1, 0);

    private boolean mTrackedOffsetSet;

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret();
        }

        return mInstance;
    }

    private Turret() {
        mPeriodicIO = new PeriodicIO();

        //Initalize subsystem devices
        mAzimuth = TalonFXFactory.createDefaultTalon(kAzimuthId);
        mAzimuth.configPeakOutputForward(kTurretPeakOutput);
        mAzimuth.configPeakOutputReverse(-kTurretPeakOutput);
        mAzimuth.setInverted(false);

        mEncoderA = new BuzzCANCoder(kTurretEncoderAId, false);
        mEncoderB = new BuzzCANCoder(kTurretEncoderBId, false);

        pidf.setOutputRange(-12, 12);
        pidf.setDeadband(1); //This only applies for the P term
    }

    public double getAngle()  {
        return mPeriodicIO.trackedPosition;
    }

    public void disable() {
        mPeriodicIO.commandMode = CommandMode.VOLTAGE;
        mPeriodicIO.command = 0;
    }

    public synchronized void setBraked(boolean braked) {
        mAzimuth.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setDemandVoltage(double demandVoltage) {
        mPeriodicIO.commandMode = CommandMode.VOLTAGE;
        mPeriodicIO.command = demandVoltage;
    }

    protected void setAngle(double angle) {
        mPeriodicIO.commandMode = CommandMode.ABS;
        mPeriodicIO.command = angle;
    }

    public enum CommandMode {
        DISABLED,
        VOLTAGE,
        ABS,
        TRACKED
    } 

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double rawAbsDiff;
        public double absPosition;
        public boolean absPositionValid;
        public double relPosition;
        public boolean relPositionValid;
        public double trackedPositionOffset;
        public double trackedPosition;
        public boolean trackedPositionValid;

        // OUTPUTS
        public double command;
        public CommandMode commandMode = CommandMode.DISABLED;
    }

    MovingAverage movingAverage = new MovingAverage(5);
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        boolean lastAbsPositionValid = mPeriodicIO.absPositionValid;
        mPeriodicIO.relPosition = 1;
        
        mPeriodicIO.rawAbsDiff = mEncoderA.getAbsoluteRevs() - mEncoderB.getAbsoluteRevs();
        double rawDiff = mPeriodicIO.rawAbsDiff - kTurretSensorOffset.get();
        if (rawDiff < 0) rawDiff += 1;
        double newAbsPosition = rawDiff * -kTurretSensorScale;
        if (newAbsPosition <= -100) newAbsPosition += kTurretSensorScale;

        movingAverage.add(newAbsPosition);
        newAbsPosition = movingAverage.getAverage();

        mPeriodicIO.absPosition = newAbsPosition;
        
        mPeriodicIO.absPositionValid = mEncoderA.getValid() && mEncoderB.getValid();

        //TODO Update
        mPeriodicIO.trackedPosition = mPeriodicIO.absPosition;

        if(!lastAbsPositionValid && mPeriodicIO.absPositionValid) {} //TODO Update
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output

        mAzimuth.set(ControlMode.Disabled, 0);
        return;

        /*
        switch(mPeriodicIO.commandMode) {
            case DISABLED:
                mAzimuth.set(ControlMode.Disabled, 0);
                break;
            case VOLTAGE:
                mAzimuth.setDemandVoltage(mPeriodicIO.command);
                break;
            case ABS:
                pidf.setSetpoint(mPeriodicIO.command);
                //Do not do integral control unless turret is within 4 degrees of the target
                if(Math.abs(mPeriodicIO.command - mPeriodicIO.absPosition) > 5) {
                    pidf.resetIntegrator();
                }
                double demand = pidf.calculate(mPeriodicIO.absPosition);
                if(mPeriodicIO.absPosition < kTurretAngleMin && demand < 0) demand = 0;
                if(mPeriodicIO.absPosition > kTurretAngleMax && demand > 0) demand = 0;
                mAzimuth.setDemandVoltage(demand);
                break;
            case TRACKED:
                break;
        }
        */
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
        SmartDashboard.putNumber("Turret Raw Diff", mPeriodicIO.rawAbsDiff);
        SmartDashboard.putNumber("Turret Abs Pos", Math.round(mPeriodicIO.absPosition * 100d) / 100d);
        SmartDashboard.putBoolean("Turret Abs Pos Valid", mPeriodicIO.absPositionValid);
        SmartDashboard.putString("Turret Mode", mPeriodicIO.commandMode.toString());
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}