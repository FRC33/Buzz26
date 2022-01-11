package frc2020;

import java.util.Optional;

import edu.wpi.first.wpilibj.Compressor;
import frc2020.auto.AutoModeExecutor;
import frc2020.auto.modes.AutoModeBase;
import lib.SubsystemManager;
//import lib.vision.AimingParameters;
import lib.loops.Looper;
import lib.util.CrashTracker;
import lib.wpilib.IRobotContainer;
import lib.wpilib.TimedRobot;

/**
 * Do not edit this class unless necessary. Edit {@link RobotContainer} instead.
 */
public class Robot extends TimedRobot {

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;

    private final Compressor mCompressor;

    private IRobotContainer mRobotContainer = new RobotContainer();
    
    Robot() {
        CrashTracker.logRobotConstruction();
        mCompressor = new Compressor();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(mRobotContainer.getSubsystems());
            
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mAutoModeSelector.updateModeCreator();

            // ROBOT CONTAINER METHOD
            mRobotContainer.robotInit();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            mEnabledLooper.stop();

            mCompressor.stop();

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            // ROBOT CONTAINER METHOD
            mRobotContainer.disabledInit();

            mDisabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            enabledInit();

            // ROBOT CONTAINER METHOD
            mRobotContainer.autonomousInit();

            mAutoModeExecutor.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            // ROBOT CONTAINER METHOD
            mRobotContainer.teleopInit();

            enabledInit();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void enabledInit() {
        mDisabledLooper.stop();
        
        mCompressor.start();
        
        // ROBOT CONTAINER METHOD
        mRobotContainer.enabledInit();

        mEnabledLooper.start();
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();

            // ROBOT CONTAINER METHOD
            mRobotContainer.testInit();
            enabledInit();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            //Output to dashboard
            mSubsystemManager.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();

            // ROBOT CONTAINER METHOD
            mRobotContainer.robotPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        try {
            mAutoModeSelector.updateModeCreator();
            
            // ROBOT CONTAINER METHOD
            mRobotContainer.disabledPeriodic();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        // ROBOT CONTAINER METHOD
        mRobotContainer.testPeriodic();
    }

    @Override
    public void teleopPeriodic() {
        try {
            // ROBOT CONTAINER METHOD
            mRobotContainer.teleopPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        // ROBOT CONTAINER METHOD
        mRobotContainer.testPeriodic();
    }
}