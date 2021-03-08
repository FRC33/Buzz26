package frc2020.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Timer;
import frc2020.subsystems.Hood.CommandMode;
import lib.drivers.BuzzTalonFX;
import lib.drivers.TalonFXFactory;
import lib.geometry.Rotation2d;
import lib.subsystems.Subsystem;

/**
 * Unlike other subystems, this is not a singleton since their will be 4 swerve modules.
 */
public class SwerveModule extends Subsystem {

    private BuzzTalonFX mDriveMotor;
    private BuzzTalonFX mSteerMotor;
    private Counter mSteerEncoder;

    private Optional<Double> mTrackedAngleOffset = Optional.empty();

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
        public int kDriveMotorId = 1; /** Falcon CAN ID */
        public int kSteerMotorId = 2; /** Falcon CAN ID */
        public int kSteerEncoderId = 0; /** DIO channel */

        public double kDriveMotorGearReduction;
        public double kDriveWheelDiameter = 6; /** in */
        //TODO add current limiting
        public double kDriveKp;
        public double kDriveKi;
        public double kDriveKd;
        public double kDriveKf;
        public double kDriveKiZone = 0;

        public double kSteerMotorGearReduction;
        public double kSteerEncoderGearReduction;
        //TODO add current limiting
        public double kSteerKp;
        public double kSteerKi;
        public double kSteerKd;
        public double kSteerKf;
        public double kSteerKiZone = 0;
        public double kSteerKv; /** Feedforward velocity */
        public double kSteerKa; /** Feedforward acceleration */
    }

    private SwerveModule(SwerveModuleConstants constants) {
        mPeriodicIO = new PeriodicIO();

        // Initalize subsystem devices
        mDriveMotor = TalonFXFactory.createDefaultTalon(constants.kDriveMotorId);
        mSteerMotor = TalonFXFactory.createDefaultTalon(constants.kSteerMotorId);
        mSteerEncoder = new Counter(constants.kSteerEncoderId);

        //TODO config device properties based on constants

        mSteerEncoder.setSemiPeriodMode(true);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        // Drive
        public double driveSupplyVoltage;
        public double driveVelocity; /** in/s */
        // Steer
        public double steerSupplyVoltage;
        public double absoluteAngle; /** Degrees from magnetic encoder */
        public double relativeAngle; /** Degrees from steer Falcon integrated sensor. Not wrapped */
        public double trackedAngle; /** Not wrapped */

        // OUTPUTS
        public double driveCommand;
        public double steerCommand; /** Wrapped -180 to 180 */
        public DriveMode driveMode;
        public SteerMode steerMode;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output
        
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

    public void setAngle(double angle) {
        mPeriodicIO.steerCommand = angle;
        mPeriodicIO.steerMode = SteerMode.ANGLE;
    }

    public void setAngle(Rotation2d rotation2d) {
        setAngle(rotation2d.getDegrees());
    }

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
