package lib.wpilib;

import lib.subsystems.Subsystem;

public interface IRobotContainer {
    Subsystem[] getSubsystems();
    void robotInit();
    void disabledInit();
    void autonomousInit();
    void teleopInit();
    void testInit();
    void enabledInit();
    void robotPeriodic();
    void disabledPeriodic();
    void autonomousPeriodic();
    void teleopPeriodic();
    void testPeriodic();
}
