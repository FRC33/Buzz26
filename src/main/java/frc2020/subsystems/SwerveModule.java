package frc2020.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import frc2020.Constants;
import lib.drivers.TalonFXFactory;
import lib.drivers.TalonSRXFactory;
import lib.geometry.Rotation2d;
import lib.subsystems.Subsystem;

/**
 * Unlike other subystems, this is not a singleton since their will be 4 swerve modules.
 */
public class SwerveModule extends Subsystem {

    private BaseTalon mDriveMotor;
    private BaseTalon mSteerMotor;
    private DutyCycle mSteerEncoder;

    private Optional<Double> mTrackedAngleOffset = Optional.empty();

    private SwerveModuleConstants mConstants;

    public enum DriveMode {
        DISABLED,
        VOLTAGE,
        VELOCITY
    }

    public enum SteerMode {
        DISABLED,
        VOLTAGE,
        ANGLE /** Motion magic */
    }

    public static class SwerveModuleConstants {
        /** Falcon CAN ID */
        public int kDriveMotorId = 1;
        /** Falcon CAN ID */
        public int kSteerMotorId = 2;
        /** DIO channel */
        public int kSteerEncoderId = 0;

        public double kDriveMotorGearReduction = 4.67;
        /** in */
        public double kDriveWheelDiameter = 3;
        //TODO add current limiting
        public double kDriveKp;
        public double kDriveKi;
        public double kDriveKd;
        public double kDriveKf;
        public double kDriveKiZone = 0;

        public double kSteerMotorGearReduction;
        //public double kSteerEncoderGearReduction;
        //TODO add current limiting
        /** The raw absolute revolutions [0, 1] when the wheel faces forward */
        public double kSteerEncoderOffset;
        public double kSteerKp;
        public double kSteerKi;
        public double kSteerKd;
        public double kSteerKf;
        public double kSteerKiZone = 0;
        /** Feedforward velocity (aka cruise velocity) */
        public double kSteerKv;
        /** Feedforward acceleration */
        public double kSteerKa;
    }

    public SwerveModule(SwerveModuleConstants constants) {
        mPeriodicIO = new PeriodicIO();

        mConstants = constants;

        // Initalize subsystem devices
        if(RobotBase.isReal()) {
            mDriveMotor = TalonFXFactory.createDefaultTalon(constants.kDriveMotorId);
            mSteerMotor = TalonFXFactory.createDefaultTalon(constants.kSteerMotorId);
        } else {
            mDriveMotor = TalonSRXFactory.createDefaultTalon(constants.kDriveMotorId);
            mSteerMotor = TalonSRXFactory.createDefaultTalon(constants.kSteerMotorId);
        }
        
        mSteerEncoder = new DutyCycle(new DigitalInput(constants.kSteerEncoderId));

        //TODO config device properties based on constants
        // Need to config sat voltage before enabling comp
        // mSteerMotor.enableVoltageCompensation(true);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        // Drive
        public double driveSupplyVoltage;
        /** in/s */
        public double driveVelocity;
        /** in */
        public double drivePosition;
        // Steer
        public double steerSupplyVoltage;
        /** Magnetic encoder position from 0 to 1, no offset */
        public double rawAbsoluteRevs;
        /** Degrees from magnetic encoder */
        public double absoluteAngle;
        /** Degrees from steer Falcon integrated sensor. Not wrapped */
        public double relativeAngle;
        /** Not wrapped */
        public double trackedAngle;

        // OUTPUTS
        public double driveCommand;
        /** Wrapped -180 to 180 */
        public double steerCommand;
        public DriveMode driveMode;
        public SteerMode steerMode;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        mPeriodicIO.driveSupplyVoltage = mDriveMotor.getBusVoltage();
        mPeriodicIO.driveVelocity = (((mDriveMotor.getSelectedSensorVelocity() / Constants.kFalconCPR) / mConstants.kDriveMotorGearReduction) * 10d) // rev/s
                                     * (mConstants.kDriveWheelDiameter * Math.PI); // Scale revs to inches
        mPeriodicIO.drivePosition = ((mDriveMotor.getSelectedSensorPosition() / Constants.kFalconCPR) / mConstants.kDriveMotorGearReduction) // rev
                                     * (mConstants.kDriveWheelDiameter * Math.PI); // Scale revs to inches
        
        mPeriodicIO.steerSupplyVoltage = mDriveMotor.getBusVoltage();

        mPeriodicIO.rawAbsoluteRevs = mSteerEncoder.getOutput();
        var absoluteRevs = mPeriodicIO.rawAbsoluteRevs - mConstants.kSteerEncoderOffset; // Subtract offset so that 0 revs = 0 degrees
        if(absoluteRevs < 0) absoluteRevs += 1; // Wrap negative values to be back inside range [0, 1]
        mPeriodicIO.absoluteAngle = (absoluteRevs - 0.5) * 360; // Scale [0, 1] to [-180, 180]

        mPeriodicIO.relativeAngle = ((mSteerMotor.getSelectedSensorPosition() / Constants.kFalconCPR) / mConstants.kSteerMotorGearReduction) // rev
                                    * 360; // Scales revs to degrees

        if(mTrackedAngleOffset.isEmpty()) {
            mTrackedAngleOffset = Optional.of(mPeriodicIO.absoluteAngle - mPeriodicIO.relativeAngle);
        }
        mPeriodicIO.trackedAngle = mPeriodicIO.relativeAngle + mTrackedAngleOffset.get();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        switch(mPeriodicIO.driveMode) {
            case VELOCITY:
                var scaledDriveCommand = ((mPeriodicIO.driveCommand / (mConstants.kDriveWheelDiameter * Math.PI)) 
                * Constants.kFalconCPR * mConstants.kDriveMotorGearReduction) / 10d;
                mDriveMotor.set(ControlMode.Velocity, scaledDriveCommand);

                break;
            case VOLTAGE:
                mDriveMotor.set(ControlMode.PercentOutput, mPeriodicIO.driveCommand / 12);

                break;
            case DISABLED:
            default:
                mDriveMotor.set(ControlMode.PercentOutput, mPeriodicIO.driveCommand);
                
                break;
        }

        switch(mPeriodicIO.steerMode) {
            case VOLTAGE:
                mSteerMotor.set(ControlMode.PercentOutput, mPeriodicIO.steerCommand / 12);

                break;
            case ANGLE:
                if (!mTrackedAngleOffset.isEmpty()) {
                    var wrappedCurrentAngle = mPeriodicIO.trackedAngle % 360;
                    if(wrappedCurrentAngle < 0) wrappedCurrentAngle += 360;
                    var adjust = mPeriodicIO.steerCommand - wrappedCurrentAngle;

                    if(Math.abs(adjust) >= 180) {
                        adjust = adjust > 0 ? 360 - adjust : 360 + adjust;
                    }

                    var newSteerCommand = wrappedCurrentAngle + adjust;

                    var steerCommandEncoderUnits = ((newSteerCommand - mTrackedAngleOffset.get()) / 360) 
                    * Constants.kFalconCPR * mConstants.kSteerMotorGearReduction; // Scales steer cmd in degs to ticks
                    mSteerMotor.set(ControlMode.MotionMagic, steerCommandEncoderUnits);
                }
                else {
                    stop();
                }

                break;
            case DISABLED:
            default:
                mSteerMotor.set(ControlMode.PercentOutput, mPeriodicIO.steerCommand);

                break;
        }
    }

    public double getVelocity() {
        return mPeriodicIO.driveVelocity;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mPeriodicIO.trackedAngle);
    }

    public void disable() {
        mPeriodicIO.driveCommand = 0;
        mPeriodicIO.driveMode = DriveMode.DISABLED;
        mPeriodicIO.steerCommand = 0;
        mPeriodicIO.steerMode = SteerMode.DISABLED;
    }

    public void setVoltage(double voltage) {
        mPeriodicIO.driveCommand = voltage;
        mPeriodicIO.driveMode = DriveMode.VOLTAGE;
    }

    public void setVelocity(double velocity) {
        mPeriodicIO.driveCommand = velocity;
        mPeriodicIO.driveMode = DriveMode.VELOCITY;
    }

    public void setSteerVoltage(double voltage) {
        mPeriodicIO.steerCommand = voltage;
        mPeriodicIO.steerMode = SteerMode.VOLTAGE;
    }

    public void setAngle(double angle) {
        mPeriodicIO.steerCommand = angle;
        mPeriodicIO.steerMode = SteerMode.ANGLE;
    }

    public void setAngle(Rotation2d rotation2d) {
        setAngle(rotation2d.getDegrees());
    }

    public void resetOffset() {
        mTrackedAngleOffset = Optional.empty();
    }

    // region Simulation
    public TalonSRXSimCollection getDriveSim() {
        return ((TalonSRX) mDriveMotor).getSimCollection();
    }

    public TalonSRXSimCollection getSteerSim() {
        return ((TalonSRX) mSteerMotor).getSimCollection();
    }

    public DutyCycleSim getEncoderSim() {
        return new DutyCycleSim(mSteerEncoder);
    }
    // endregion

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
    }
    
}