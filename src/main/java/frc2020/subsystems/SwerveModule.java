package frc2020.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
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

    private String mName;
    
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
        public String kName = "";

        /** Falcon CAN ID */
        public int kDriveMotorId = 1;
        /** Falcon CAN ID */
        public int kSteerMotorId = 2;
        /** DIO channel */
        public int kSteerEncoderId = 0;

        public boolean kDriveInverted = false;
        public double kDriveMotorGearReduction = 4.67;
        /** in */
        public double kDriveWheelDiameter = Units.inchesToMeters(3);
        //TODO add current limiting
        /** Seconds */
        public double kDriveRamp = 0.3;
        public double kDriveKp = 0.1;
        public double kDriveKi = 0;
        public double kDriveKd = 0;
        public double kDriveKf = (1023 * 0.76) / 14253;
        public double kDriveKiZone = 0;

        public boolean kSteerInverted = false;
        public double kSteerMotorGearReduction = 1;
        //TODO add current limiting
        /** The raw absolute revolutions [0, 1] when the wheel faces forward */
        public double kSteerEncoderOffset = 0;
        public double kSteerKp = 0.1;
        public double kSteerKi = 0;
        public double kSteerKd = 0;
        public double kSteerKf = (1023 * 0.4350) / 8841;
        public double kSteerKiZone = 0;
        /** Feedforward velocity (aka cruise velocity) */
        public double kSteerKv = 12000;
        /** Feedforward acceleration */
        public double kSteerKa = 12000 * 4;
        public int kSteerSCurveStrength = 1;
    }


    public SwerveModule(SwerveModuleConstants constants) {
        mPeriodicIO = new PeriodicIO();

        mConstants = constants;

        mName = constants.kName;

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
        mDriveMotor.setInverted(constants.kDriveInverted);
        mDriveMotor.setNeutralMode(NeutralMode.Brake);
        mDriveMotor.enableVoltageCompensation(true);
        mDriveMotor.configVoltageCompSaturation(12.0);
        mDriveMotor.configOpenloopRamp(constants.kDriveRamp);
        mDriveMotor.configClosedloopRamp(constants.kDriveRamp);
        mDriveMotor.config_kP(0, constants.kDriveKp);
        mDriveMotor.config_kI(0, constants.kDriveKi);
        mDriveMotor.config_kD(0, constants.kDriveKd);
        mDriveMotor.config_kF(0, constants.kDriveKf);
        mDriveMotor.config_IntegralZone(0, constants.kDriveKiZone);

        mSteerMotor.setInverted(constants.kSteerInverted);
        mSteerMotor.setNeutralMode(NeutralMode.Brake);
        mSteerMotor.enableVoltageCompensation(true);
        mSteerMotor.configVoltageCompSaturation(12.0);
        mSteerMotor.configMotionCruiseVelocity(constants.kSteerKv);
        mSteerMotor.configMotionAcceleration(constants.kSteerKa);
        mSteerMotor.configMotionSCurveStrength(constants.kSteerSCurveStrength);
        mSteerMotor.config_kP(0, constants.kSteerKp);
        mSteerMotor.config_kI(0, constants.kSteerKi);
        mSteerMotor.config_kD(0, constants.kSteerKd);
        mSteerMotor.config_kF(0, constants.kSteerKf);
        mSteerMotor.config_IntegralZone(0, constants.kSteerKiZone);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        // Drive
        public double driveSupplyVoltage;
        public double driveCommandVoltage;
        public double driveRawVelocity;
        /** in/s */
        public double driveVelocity;
        /** in */
        public double drivePosition;
        // Steer
        public double steerSupplyVoltage;
        public double steerCommandVoltage;
        public double steerRawVelocity;
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
        public DriveMode driveMode = DriveMode.DISABLED;
        public SteerMode steerMode = SteerMode.DISABLED;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        mPeriodicIO.driveSupplyVoltage = mDriveMotor.getBusVoltage();
        mPeriodicIO.driveCommandVoltage = mDriveMotor.getMotorOutputVoltage();
        mPeriodicIO.driveRawVelocity = mDriveMotor.getSelectedSensorVelocity();
        mPeriodicIO.driveVelocity = (((mPeriodicIO.driveRawVelocity / Constants.kFalconCPR) / mConstants.kDriveMotorGearReduction) * 10d) // rev/s
                                     * (mConstants.kDriveWheelDiameter * Math.PI); // Scale revs to inches
        mPeriodicIO.drivePosition = ((mDriveMotor.getSelectedSensorPosition() / Constants.kFalconCPR) / mConstants.kDriveMotorGearReduction) // rev
                                     * (mConstants.kDriveWheelDiameter * Math.PI); // Scale revs to inches
        
        mPeriodicIO.steerSupplyVoltage = mSteerMotor.getBusVoltage();
        mPeriodicIO.steerCommandVoltage = mSteerMotor.getMotorOutputVoltage();
        mPeriodicIO.steerRawVelocity = mSteerMotor.getSelectedSensorVelocity();

        mPeriodicIO.rawAbsoluteRevs = mSteerEncoder.getOutput();
        var absoluteRevs = mPeriodicIO.rawAbsoluteRevs - mConstants.kSteerEncoderOffset; // Subtract offset so that 0 revs = 0 degrees
        if(absoluteRevs < 0) absoluteRevs += 1; // Wrap negative values to be back inside range [0, 1]
        mPeriodicIO.absoluteAngle = absoluteRevs * -360; // Scale [0, 1] to [0, -360]
        mPeriodicIO.absoluteAngle = Rotation2d.fromDegrees(mPeriodicIO.absoluteAngle).getDegrees();

        mPeriodicIO.relativeAngle = ((mSteerMotor.getSelectedSensorPosition() / Constants.kFalconCPR) / mConstants.kSteerMotorGearReduction) // rev
                                    * 360; // Scales revs to degrees

        if(mTrackedAngleOffset.isEmpty()) {
            mTrackedAngleOffset = Optional.of(mPeriodicIO.absoluteAngle - mPeriodicIO.relativeAngle);
        }
        mPeriodicIO.trackedAngle = mPeriodicIO.relativeAngle + mTrackedAngleOffset.get();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        double raw_error = 0;
        if(mPeriodicIO.steerMode == SteerMode.ANGLE) {
            Rotation2d current = Rotation2d.fromDegrees(mPeriodicIO.trackedAngle);

            raw_error = current.distance(Rotation2d.fromDegrees(mPeriodicIO.steerCommand));
            if(Math.abs(raw_error) > Math.PI) {
                raw_error -= (Math.PI * 2 * Math.signum(raw_error));
            }
        }
        if(mPeriodicIO.driveMode == DriveMode.VELOCITY && mPeriodicIO.steerMode == SteerMode.ANGLE) {
            if(mPeriodicIO.driveVelocity <= Units.inchesToMeters(30)) {
                if(Math.abs(raw_error) > Math.PI / 2) {
                    mPeriodicIO.driveCommand *= -1;
                    raw_error -= Math.PI * Math.signum(raw_error);
                }
            }
        }

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
                    

                    double final_setpoint = mPeriodicIO.trackedAngle + Units.radiansToDegrees(raw_error);                  

                    var steerCommandEncoderUnits = ((final_setpoint - mTrackedAngleOffset.get()) / 360) 
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

    public PeriodicIO getPeriodicIO() {
        return mPeriodicIO;
    }

    public double getVelocity() {
        return mPeriodicIO.driveVelocity;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mPeriodicIO.trackedAngle);
    }

    public edu.wpi.first.wpilibj.geometry.Rotation2d getAngleWPI() {
        return edu.wpi.first.wpilibj.geometry.Rotation2d.fromDegrees(mPeriodicIO.trackedAngle);
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

    public void setBraked(boolean braked) {
        mDriveMotor.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
        mSteerMotor.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocity(), getAngleWPI());
    }

    public synchronized void resetOffset() {
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
        SmartDashboard.putNumber(mName + " Drive Kf", (1023 * (mPeriodicIO.driveCommandVoltage / 12)) / mPeriodicIO.driveRawVelocity);
        SmartDashboard.putNumber(mName + " Velocity", mPeriodicIO.driveVelocity);
        SmartDashboard.putNumber(mName + " Target Velocity", mPeriodicIO.driveCommand);

        SmartDashboard.putNumber(mName + " Steer Kf", (1023 * (mPeriodicIO.steerCommandVoltage / 12)) / mPeriodicIO.steerRawVelocity);
        SmartDashboard.putNumber(mName + " Raw Revs", mPeriodicIO.rawAbsoluteRevs);
        SmartDashboard.putNumber(mName + " Abs Angle", mPeriodicIO.absoluteAngle);
        SmartDashboard.putNumber(mName + " Rel Angle", mPeriodicIO.relativeAngle);
        SmartDashboard.putNumber(mName + " Tracked Angle", mPeriodicIO.trackedAngle);
        SmartDashboard.putNumber(mName + " Target Angle", mPeriodicIO.steerCommand);
        SmartDashboard.putNumber(mName + " Rot2d Angle", getAngle().getDegrees());
    }
    
}