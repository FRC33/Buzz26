package frc2020.subsystems;

import static frc2020.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.drivers.BuzzTalonFX;
import lib.drivers.TalonFXFactory;
import lib.subsystems.Subsystem;

public class Shooter extends Subsystem {
    private static Shooter mInstance;

    private BuzzTalonFX mShooterA;
    private BuzzTalonFX mShooterB;

    public enum CommandMode {
        DISABLED,
        VOLTAGE,
        RPM
    }

    public synchronized static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }

    private Shooter() {
        mPeriodicIO = new PeriodicIO();

        // Initalize subsystem devices
        mShooterA = TalonFXFactory.createDefaultTalon(kShooterAId);
        mShooterB = TalonFXFactory.createDefaultTalon(kShooterBId);
        configShooterMotor(mShooterA);
        configShooterMotor(mShooterB);
    }

    private void configShooterMotor(BuzzTalonFX motor) {
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setBuzzTalonScalars(kShooterGearReduction, kShooterDiameter, kFalconCPR);
        motor.configOpenloopRamp(0);
        motor.configClosedloopRamp(kShooterRampRate);
        motor.configPIDF(kShooterSlotIdx, kShooterKp, kShooterKi, kShooterKd, kShooterKf);

        motor.enableVoltageCompensation(false);
        // TODO Renable?
        //motor.configVoltageCompSaturation(11.5);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double rpmA;
        public double rpmB;
        public double voltageA;
        public double voltageB;

        // OUTPUTS
        public double command;
        public CommandMode commandMode = CommandMode.DISABLED;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        mPeriodicIO.rpmA = mShooterA.getRPM();
        mPeriodicIO.rpmB = mShooterB.getRPM();
        mPeriodicIO.voltageA = mShooterA.getMotorOutputVoltage();
        mPeriodicIO.voltageB = mShooterA.getMotorOutputVoltage();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output
        
        switch(mPeriodicIO.commandMode) {
            case DISABLED:
                mShooterA.set(ControlMode.PercentOutput, 0);
                mShooterB.set(ControlMode.PercentOutput, 0);
                break;
            case VOLTAGE:
                mShooterA.setDemandVoltage(mPeriodicIO.command);
                mShooterB.setDemandVoltage(mPeriodicIO.command);
                break;
            case RPM:
                mShooterA.setRPM(mPeriodicIO.command);
                mShooterB.setRPM(mPeriodicIO.command);
                break;
        }
    }

    public double getRPM() {
        return (mPeriodicIO.rpmA + mPeriodicIO.rpmB) / 2;
    }

    public double getVoltage() {
        return (mPeriodicIO.voltageA + mPeriodicIO.voltageB) / 2;
    }

    public void setDemand(double demand) {
        mPeriodicIO.command = demand;
        mPeriodicIO.commandMode = CommandMode.VOLTAGE;
    }

    public void setTargetRPM(double target) {
        mPeriodicIO.command = target;
        mPeriodicIO.commandMode = CommandMode.RPM;
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
        SmartDashboard.putNumber("Shooter Current RPM", getRPM());
        SmartDashboard.putNumber("Shooter Command", mPeriodicIO.command);
        SmartDashboard.putString("Shooter Command Mode", mPeriodicIO.commandMode.toString());
        SmartDashboard.putNumber("Shooter A Voltage", mPeriodicIO.voltageA);
        SmartDashboard.putNumber("Shooter B Voltage", mPeriodicIO.voltageB);
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}