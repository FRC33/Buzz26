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

    private BuzzTalonFX mIntake;
    private BuzzTalonFX mIndexer;
    private DoubleSolenoid mIntakeSolenoid;

    private DelayedBoolean mIntakeStalledDelayedBoolean = new DelayedBoolean(Timer.getFPGATimestamp(), kIntakeStallTime);
    private DelayedBoolean mIndexerStalledDelayedBoolean = new DelayedBoolean(Timer.getFPGATimestamp(), kIntakeStallTime);

    public synchronized static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }

        return mInstance;
    }

    private Intake() {
        mPeriodicIO = new PeriodicIO();

        // Initalize subsystem devices
        mIntake = TalonFXFactory.createDefaultTalon(kIntakeId);
        mIndexer = TalonFXFactory.createDefaultTalon(kIndexerId);
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
        public boolean intakeDeploy;
        public double indexerDemand;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        mPeriodicIO.intakeStalled =
            mIntakeStalledDelayedBoolean.update(getTimestamp(), mIntake.getStatorCurrent() > Constants.kIntakeStallCurrent) ||
            mIndexerStalledDelayedBoolean.update(getTimestamp(), mIndexer.getStatorCurrent() > Constants.kIntakeStallCurrent);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output
        mIntake.setDemandVoltage(mPeriodicIO.intakeDemand);
        mIndexer.setDemandVoltage(mPeriodicIO.intakeDemand);
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

    public void setIndexer(double voltage) {
        mPeriodicIO.indexerDemand = voltage;
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
        SmartDashboard.putNumber("Brush Demand", mPeriodicIO.indexerDemand);
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}