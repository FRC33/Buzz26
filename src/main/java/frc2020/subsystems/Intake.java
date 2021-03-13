package frc2020.subsystems;

import static frc2020.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.Constants;
import lib.drivers.BuzzTalonFX;
import lib.drivers.BuzzTalonSRX;
import lib.drivers.TalonFXFactory;
import lib.drivers.TalonSRXFactory;
import lib.subsystems.Subsystem;
import lib.util.DelayedBoolean;

public class Intake extends Subsystem {
    private static Intake mInstance;

    private BuzzTalonFX mIntake1;
    //private BuzzTalonFX mIntake2;
    private BuzzTalonSRX mInfeeder;
    private BuzzTalonSRX mBrush;
    private DoubleSolenoid mIntakeSolenoid;

    private DelayedBoolean mIntake1StalledDelayedBoolean = new DelayedBoolean(Timer.getFPGATimestamp(), kIntakeStallTime);
    private DelayedBoolean mIntake2StalledDelayedBoolean = new DelayedBoolean(Timer.getFPGATimestamp(), kIntakeStallTime);

    public synchronized static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }

        return mInstance;
    }

    private Intake() {
        mPeriodicIO = new PeriodicIO();

        // Initalize subsystem devices
        mIntake1 = TalonFXFactory.createDefaultTalon(kIntake1Id);
        mIntake1.setInverted(true);
        mIntake1.configOpenloopRamp(0.1);

        mIntake2 = TalonFXFactory.createDefaultTalon(kIntake2Id);
        mIntake2.setInverted(true);
        mIntake2.configOpenloopRamp(0.1);

        mInfeeder = TalonSRXFactory.createDefaultTalon(kInfeederId);
        
        mBrush = TalonSRXFactory.createDefaultTalon(kBrushId);
        mBrush.setInverted(true);
        mBrush.setNeutralMode(NeutralMode.Brake);

        mIntakeSolenoid = new DoubleSolenoid(kIntakeForwardId, kIntakeReverseId);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public boolean intakeStalled;
        public boolean ballEntered;
        public boolean ballStaged;

        // OUTPUTS
        public double intakeDemand;
        public double infeederDemand;
        public double brushDemand;
        public boolean intakeDeploy;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        mPeriodicIO.intakeStalled =
            mIntake1StalledDelayedBoolean.update(getTimestamp(), mIntake1.getStatorCurrent() > Constants.kIntakeStallCurrent) ||
            mIntake2StalledDelayedBoolean.update(getTimestamp(), mIntake2.getStatorCurrent() > Constants.kIntakeStallCurrent);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output
        mIntake1.setDemandVoltage(mPeriodicIO.intakeDemand);
        mIntake2.setDemandVoltage(mPeriodicIO.intakeDemand);
        mInfeeder.setDemandVoltage(mPeriodicIO.infeederDemand);
        mBrush.setDemandVoltage(mPeriodicIO.brushDemand);
        mIntakeSolenoid.set(mPeriodicIO.intakeDeploy ? Value.kReverse : Value.kForward);
    }

    public boolean isIntakeDeployed() {
        return mPeriodicIO.intakeDeploy;
    }

    public boolean isStalled() {
        return mPeriodicIO.intakeStalled;
    }

    public void setIntake(double voltage) {
        mPeriodicIO.intakeDemand = voltage;
    }

    public void setInfeeder(double voltage) {
        mPeriodicIO.infeederDemand = voltage;
    }

    public void setBrush(double voltage) {
        mPeriodicIO.brushDemand = voltage;
    }

    public void setIntakeDeploy(boolean deploy) {
        mPeriodicIO.intakeDeploy = deploy;
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
        SmartDashboard.putNumber("Brush Demand", mPeriodicIO.brushDemand);
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}