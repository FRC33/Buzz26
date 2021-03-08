package frc2020.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Timer;
import lib.drivers.BuzzTalonFX;
import lib.drivers.TalonFXFactory;
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
        ANGLE
    }

    public static class SwerveModuleConstants {
        public int kDriveMotorId = 1; /** Falcon CAN ID */
        public int kSteerMotorId = 2; /** Falcon CAN ID */
        public int kSteerEncoderId = 0; /** DIO channel */

        public double kDriveMotorGearReduction;

        public double kSteerMotorGearReduction;
        
        public double kSteerEncoderGearReduction;
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
        public double driveVelocity;
        // Steer
        public double absoluteAngle;
        public double relativeAngle;
        public double trackedAngle;

        // OUTPUTS
        public double driveCommand;
        public double steerCommand;
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
