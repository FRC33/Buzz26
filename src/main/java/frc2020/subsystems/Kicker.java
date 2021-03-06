package frc2020.subsystems;

import static frc2020.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import lib.drivers.BuzzTalonSRX;
import lib.drivers.TalonSRXFactory;
import lib.subsystems.Subsystem;

public class Kicker extends Subsystem {
    private static Kicker mInstance;

    private BuzzTalonSRX mKickerMotor;

    public enum CommandMode {
        DISABLED,
        VOLTAGE
    }

    public synchronized static Kicker getInstance() {
        if (mInstance == null) {
            mInstance = new Kicker();
        }

        return mInstance;
    }

    private Kicker() {
        mPeriodicIO = new PeriodicIO();

        // Initalize subsystem devices
        mKickerMotor = TalonSRXFactory.createDefaultTalon(kKickerId);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;

        // OUTPUTS
        public double command;
        public CommandMode commandMode = CommandMode.DISABLED;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output

        switch(mPeriodicIO.commandMode) {
            case DISABLED:
                mKickerMotor.set(ControlMode.Disabled, 0);
                break;
            case VOLTAGE:
                mKickerMotor.setDemandVoltage(mPeriodicIO.command);
                break;
        }
    }

    public void disable() {
        mPeriodicIO.command = 0;
        mPeriodicIO.commandMode = CommandMode.DISABLED;
    }

    public void setDemand(double demand) {
        mPeriodicIO.command = demand;
        mPeriodicIO.commandMode = CommandMode.VOLTAGE;
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

    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}