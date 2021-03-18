package frc2020.subsystems;

import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import frc2020.subsystems.SwerveModule.SwerveModuleConstants;
import lib.SubsystemManager;
import lib.loops.Looper;

public class SwerveModuleTest {

    private static SwerveModule swerveModule;
    private static DutyCycleSim encoderSim;
    
    private static SubsystemManager subsystemManager = SubsystemManager.getInstance();
    private static final Looper enabledLooper = new Looper();
    private static final Looper disabledLooper = new Looper();

    @Ignore
    @BeforeClass
    public static void beforeClass() {
        assert HAL.initialize(500, 0);

        SwerveModuleConstants constants = new SwerveModule.SwerveModuleConstants();
        constants.kDriveWheelDiameter = 6;
        constants.kDriveMotorGearReduction = 6.666;
        constants.kSteerEncoderOffset = 0;
        constants.kSteerMotorGearReduction = 12.333;
        swerveModule = new SwerveModule(constants);

        encoderSim = swerveModule.getEncoderSim();

        subsystemManager.setSubsystems(swerveModule);
        subsystemManager.registerEnabledLoops(enabledLooper);
        subsystemManager.registerDisabledLoops(disabledLooper);

        
    }

    @Ignore
    @Test
    public void testReads() throws InterruptedException {
        enabledLooper.stop();
        disabledLooper.start();

        var vel = (((12 * 6.666) / (6 * Math.PI)) / 10) * 2048;
        swerveModule.getDriveSim().setQuadratureVelocity((int)vel);
        Thread.sleep(500);
        assertEquals(swerveModule.getVelocity(), 12, 0.5);

        var period = 0.5;
        swerveModule.getEncoderSim().setOutput(period);
        var pos = ((30.0 * 12.333) / 360) * 2048;
        swerveModule.getSteerSim().setQuadratureRawPosition((int)pos);
        Thread.sleep(500);
        swerveModule.resetOffset();
        Thread.sleep(500);

        var pos2 = ((270.0 * 12.333) / 360) * 2048;
        swerveModule.getSteerSim().setQuadratureRawPosition((int)pos2);
        Thread.sleep(500);

        assertEquals(swerveModule.getAngle().getDegrees(), -120.0, 0.1);
    }
}
