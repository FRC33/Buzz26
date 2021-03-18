package frc2020.subsystems;

import static frc2020.Constants.*;

import java.util.Optional;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.drivers.BuzzCANCoder;
import lib.drivers.SmartServo;
import lib.subsystems.Subsystem;
import lib.util.SynchronousPIDF;
import lib.util.Util;

public class Hood extends Subsystem {
    private static Hood mInstance;

    private SmartServo mLeftServo;
    private SmartServo mRightServo;
    private BuzzCANCoder mEncoder;

    private SynchronousPIDF pidf = new SynchronousPIDF(kHoodKp, kHoodKi, kHoodKd);

    private Optional<Double> mTrackedPositionOffset = Optional.empty();

    private double mDesiredPercent;

    public enum CommandMode {
        DISABLED,
        PERCENT,
        ANGLE
    }

    public synchronized static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }

        return mInstance;
    }

    private Hood() {
        mPeriodicIO = new PeriodicIO();

        // Initalize subsystem devices
        boolean invert = true;
        mLeftServo = new SmartServo(kHoodAId, invert);
        mRightServo = new SmartServo(kHoodBId, !invert);

        mEncoder = new BuzzCANCoder(kHoodEncoderId, true);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double rawAbsPosition;
        public double absPosition;
        public double relPosition;
        public double trackedPosition;

        // OUTPUTS
        public double command;
        public CommandMode commandMode = CommandMode.DISABLED;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        double absRevs = mEncoder.getAbsoluteRevs();
        //mPeriodicIO.rawAbsPosition = absRevs <= 0.5 ? 1 + absRevs : absRevs;
        mPeriodicIO.rawAbsPosition = absRevs;
        mPeriodicIO.absPosition = (((mPeriodicIO.rawAbsPosition - kHoodSensorOffset.get()) / kHoodEncoderReduction) * 360) + kHoodAngleOffset;
        mPeriodicIO.relPosition = (mEncoder.getRevs() / kHoodEncoderReduction) * 360;
        if(mTrackedPositionOffset.isEmpty()) {
            mTrackedPositionOffset = Optional.of(mPeriodicIO.absPosition - mPeriodicIO.relPosition);
        }
        mPeriodicIO.trackedPosition = mPeriodicIO.relPosition + mTrackedPositionOffset.get();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output
        mDesiredPercent = calculateDesiredPercent();
        mLeftServo.set(mDesiredPercent);
        mRightServo.set(mDesiredPercent);
    }

    public double getAngle()  {
        return mPeriodicIO.trackedPosition;
    }

    public void disable() {
        mPeriodicIO.commandMode = CommandMode.DISABLED;
    }

    public void setPercent(double percent) {
        mPeriodicIO.commandMode = CommandMode.PERCENT;
        mPeriodicIO.command = percent;
    }

    public void setAngle(double angle) {
        mPeriodicIO.commandMode = CommandMode.ANGLE;
        mPeriodicIO.command = angle;
    }

    public void forceDisable() {
        mLeftServo.set(0);
        mRightServo.set(0);
    }

    private double mLastSetpoint = 0;
    private double calculateDesiredPercent() {
        switch(mPeriodicIO.commandMode) {
            case DISABLED:
                return 0.0;
            case PERCENT:
                return mPeriodicIO.command;
            case ANGLE:
                // Reset I if target angle changed
                if(mLastSetpoint != mPeriodicIO.command) pidf.resetIntegrator();
                mLastSetpoint = mPeriodicIO.command;

                // Set setpoint
                pidf.setSetpoint(mPeriodicIO.command);

                // Calculate output from PID controller
                double demand = (pidf.calculate(mPeriodicIO.trackedPosition) / RobotController.getVoltage6V());
                
                // 0.05 is hold percent
                //demand += 0.07 * Math.sin((mPeriodicIO.trackedPosition / 180.0) * Math.PI);
                
                // Reset I if not within certain error
                if(Math.abs(pidf.getError()) > kHoodKiZone) pidf.resetIntegrator();

                // Soft limits
                if(mPeriodicIO.trackedPosition < kHoodAngleMin && demand < 0) demand = 0;
                if(mPeriodicIO.trackedPosition > kHoodAngleMax && demand > 0) demand = 0;

                return demand;
            default:
                return 0.0;
        }
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public synchronized void stop() {
        setPercent(0);
        // Ensure that the values are set (this is important because sometimes these servos moved when disabled)
        mLeftServo.set(0);
        mRightServo.set(0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Hood Command Mode", mPeriodicIO.commandMode.toString());
        SmartDashboard.putNumber("Hood Command", Util.round(mPeriodicIO.command));
        SmartDashboard.putNumber("Hood Desired Percent", Util.round(mDesiredPercent));
        SmartDashboard.putNumber("Hood Raw", Util.round(mPeriodicIO.rawAbsPosition));
        SmartDashboard.putNumber("Hood Abs", Util.round(mPeriodicIO.absPosition));
        SmartDashboard.putNumber("Hood Tracked", Util.round(mPeriodicIO.trackedPosition));
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}