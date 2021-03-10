package frc2020.subsystems;

import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.hal.HAL;
import frc2020.subsystems.SwerveModule.SwerveModuleConstants;
import lib.SubsystemManager;
import lib.loops.Looper;

public class SwerveModuleTest {

    SwerveModule swerveModule;
    
    SubsystemManager subsystemManager = SubsystemManager.getInstance();
    private final Looper enabledLooper = new Looper();
    private final Looper disabledLooper = new Looper();

    @Before
    public void before() {
        assert HAL.initialize(500, 0);

        SwerveModuleConstants constants = new SwerveModule.SwerveModuleConstants();
        constants.kDriveWheelDiameter = 6;
        constants.kDriveMotorGearReduction = 6.666;
        swerveModule = new SwerveModule(constants);

        subsystemManager.setSubsystems(swerveModule);
        subsystemManager.registerEnabledLoops(enabledLooper);
        subsystemManager.registerDisabledLoops(disabledLooper);
    }

    @Test
    public void testReads() throws InterruptedException {
        enabledLooper.stop();
        disabledLooper.start();

        var vel = (((12 * 6.666) / (6 * Math.PI)) / 10) * 2048;
        swerveModule.getDriveSim().setQuadratureVelocity((int)vel);
        Thread.sleep(500);
        assertEquals(swerveModule.getVelocity(), 12, 0.5);
    }
}
