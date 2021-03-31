package frc2020;

import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.wpilib.TimedRobot;
import lib.util.*;
//import lib.vision.AimingParameters;
import lib.loops.*;
import lib.subsystems.*;
import lib.SubsystemManager;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc2020.subsystems.Drive;
import frc2020.subsystems.Intake;
import frc2020.subsystems.Inventory;
import frc2020.subsystems.Feeder;
import frc2020.subsystems.LED;
import frc2020.subsystems.Limelight;
import frc2020.subsystems.Pixy;
import frc2020.subsystems.RobotStateEstimator;
import frc2020.subsystems.Hood;
import frc2020.subsystems.Shooter;
import frc2020.subsystems.Superstructure;
import frc2020.subsystems.SwerveModule;
import frc2020.auto.AutoModeExecutor;
import frc2020.auto.modes.AutoModeBase;
import frc2020.hmi.HMI;
import frc2020.paths.TrajectoryRegistry;
import frc2020.statemachines.ClimberStateMachine;
import frc2020.statemachines.SuperstructureStateMachine;
import frc2020.statemachines.SuperstructureStateMachine.SystemState;
import frc2020.states.LEDState;

import java.util.Optional;

public class Robot extends TimedRobot {

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    //Subsystems

    private final Drive mDrive = Drive.getInstance();
    
    //private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    
    private final Intake mIntake = Intake.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Feeder mFeeder = Feeder.getInstance();

    private final Inventory mInventory = Inventory.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final Limelight mLimelight = Limelight.getInstance();
    private final LED mLED = LED.getInstance();
    private final Pixy mPixy = Pixy.getInstance();

    private final Compressor mCompressor;

    private final HMI mHMI = HMI.getInstance();

    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;

    private TrajectoryRegistry mTrajectoryRegistry = TrajectoryRegistry.getInstance();

    private boolean mCoastDrive = true;
    
    Robot() {
        CrashTracker.logRobotConstruction();
        mCompressor = new Compressor();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                //mRobotStateEstimator,
                mDrive,
                mDrive.getSwerveModules()[0],
                mDrive.getSwerveModules()[1],
                mDrive.getSwerveModules()[2],
                mDrive.getSwerveModules()[3],
                mIntake,
                mHood,
                mShooter,
                mFeeder,
                mInventory,
                mSuperstructure,
                mLimelight,
                mLED,
                mPixy
            );
            
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mAutoModeSelector.updateModeCreator();

            mTrajectoryRegistry.load("Slalom", "Barrel", "Bounce1");

            if(!SmartDashboard.containsKey("Disable Shooter")) {
                SmartDashboard.putBoolean("Disable Shooter", true);
            }

            //CameraServer.getInstance().startAutomaticCapture();
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
            mDrive.setBraked(true);
            mHood.forceDisable();

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();

            SmartDashboard.putBoolean("autoInit", false);
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
            mAutoModeExecutor.start();
            SmartDashboard.putBoolean("autoInit", true);
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

            SmartDashboard.putBoolean("autoInit", true);
            enabledInit();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void enabledInit() {
        mDisabledLooper.stop();
        
        mCompressor.start();
        mDrive.setBraked(true);

        mSuperstructure.setDisabled(false);
        mDrive.setDisabled(false);

        mCoastDrive = false;

        mEnabledLooper.start();
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            testTimer.reset();
            testTimer.start();
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
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public static boolean disableShooter = false;
    @Override
    public void disabledPeriodic() {
        try {
            mAutoModeSelector.updateModeCreator();
            
            //Reset Field to Vehicle
            var robotState = RobotState.getInstance();
            robotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.resetGyro();
            mDrive.resetOdometry();

            mSuperstructure.resetStateMachine();

            intakeOn = false;
            aimManual = false;
            enableFlywheel = false;

            disableShooter = SmartDashboard.getBoolean("Disable Shooter", false);

            if(mDrive.getLinearVelocity() < Units.inchesToMeters(5)) {
                mCoastDrive = true;
            }
            mDrive.setBraked(!mCoastDrive);
            for(SwerveModule swerveModule : mDrive.getSwerveModules()) {
                swerveModule.resetOffset();
            }

            mHood.forceDisable();

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
        mDrive.setBraked(true);
    }

    @Override
    public void teleopPeriodic() {
        try {
            manualControl();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    //Degrees that all shot altitudes change by when the operator wants to increments or decrements it
    private static double deltaAltitudeOffset = 1;

    private LatchedBoolean lToggleCollect = new LatchedBoolean();
    private LatchedBoolean lBlowOff = new LatchedBoolean();
    private LatchedBoolean lAltitudeInc = new LatchedBoolean();
    private LatchedBoolean lAltitudeDec = new LatchedBoolean();

    private boolean intakeOn = false;
    private boolean aimManual = false;
    private boolean enableFlywheel = false;

    double lastTimestamp = Timer.getFPGATimestamp();
    public void manualControl() {
        double timestamp = Timer.getFPGATimestamp();
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        //Update latched buttons
        boolean toggleCollect = lToggleCollect.update(mHMI.getToggleCollect());
        boolean blowOff = lBlowOff.update(!mHMI.getBlow());
        boolean altitudeInc = lAltitudeInc.update(mHMI.getAltitudeInc());
        boolean altitudeDec = lAltitudeDec.update(mHMI.getAltitudeDec());

        //Shooting location
        ShootingLocation.Location shootingLocation = mHMI.getShootingLocation();
        boolean isShootingLocation = shootingLocation != ShootingLocation.Location.NONE;
        if(isShootingLocation) {
            mSuperstructure.setWantedShootingLocation(shootingLocation);
        } else if(mHMI.getClearShot()) {
            mSuperstructure.setWantedShootingLocation(ShootingLocation.Location.NONE);
        }

        //Drive
        boolean search = mSuperstructure.systemStateIsLimelight() && 
                        (mHood.getAngle() > 45.0 || shootingLocation == ShootingLocation.Location.GREEN);
        mDrive.setTeleOpInputs(
            mHMI.getThrottle(),
            mHMI.getStrafe(),
            mHMI.getSteer(),
            mHMI.getDriver().getBButton(),
            false,
            mHMI.getDriver().getAButton());

        if(isShootingLocation) {
            enableFlywheel = true;
        } else if(mHMI.getClearShot()) {
            mSuperstructure.setWantIdle();
            enableFlywheel = false;
            aimManual = false;
        }

        // ----- Superstructure Wanted Action ------
        // ----- Intake -----
        // Automatically toggle intake off once there are 3 balls
        if(mSuperstructure.getSystemState() == SuperstructureStateMachine.SystemState.INTAKE_FINISH && 
            intakeOn
        ) {
            mSuperstructure.setWantIdle();
            intakeOn = false;
        }
        if(toggleCollect && !enableFlywheel) {
            if(intakeOn) {
                mSuperstructure.setWantIdle();
                intakeOn = false;
            } else {
                mSuperstructure.setWantIntakeOn();
                intakeOn = true;
            }
        } else {
            // ----- Blow -----
            if(mHMI.getBlow()) {
                mSuperstructure.setWantBlow();
            } else if(blowOff) {
                //This handles what happens when the blow button is released, depending on if intake is toggeled or not.
                if(intakeOn) {
                    //Return to intaking if intake is still toggled on
                    mSuperstructure.setWantIntakeOn();
                } else {
                    //Otherwise just turn off blow
                    mSuperstructure.setWantIdle();
                }
            } else {
                // ----- Aim -----
                if(enableFlywheel && !mHMI.getAim()) {
                    mSuperstructure.setWantEnableFlywheel();
                    intakeOn = false;
                } else if(enableFlywheel && mHMI.getAim()) {
                    if(!mHMI.getShoot()) {
                        if((mHMI.getTurretManual() != 0 || aimManual) && mHMI.getAim()) {
                            mSuperstructure.setWantAimManual();
                            aimManual = true;
                        } else {
                            mSuperstructure.setWantAimLimelight();
                        }
                    } else {
                        // ----- Shoot -----
                        mSuperstructure.setWantShoot();
                    }
                }
            }
        }

        //Altitude offset
        if(altitudeInc) {
            //Increment
            mSuperstructure.changeAltitudeOffset(deltaAltitudeOffset);
        } else if(altitudeDec) {
            //Decrement
            mSuperstructure.changeAltitudeOffset(-deltaAltitudeOffset);
        }

        //Manual brush
        if(mHMI.getBrushForward()) {
            mSuperstructure.setBrushOverride(false);
        } else if(mHMI.getBrushBackward()) {
            mSuperstructure.setBrushOverride(true);
        } else {
            mSuperstructure.stopBrushOverride();
        }

        /*
        //Climber
        ClimberStateMachine.WantedAction climberWantedAction = ClimberStateMachine.WantedAction.STOP;
        if(mHMI.getMastUp()) {
            climberWantedAction = ClimberStateMachine.WantedAction.MAST_UP;
        } else if(mHMI.getMastDown()) {
            climberWantedAction = ClimberStateMachine.WantedAction.MAST_DOWN;
        } else if(mHMI.getWinchIn()) {
            climberWantedAction = ClimberStateMachine.WantedAction.WINCH_IN;
        } else if(mHMI.getWinchOut()) {
            climberWantedAction = ClimberStateMachine.WantedAction.WINCH_OUT;
        }
        mClimber.setWantedAction(climberWantedAction);

        // LEDs
        
        // TODO clean up
        /*
        if(mClimber.getPartyMode()) {
            mLED.setState(Color.WHITE, 0, true);
        } else {
            if(mSuperstructure.systemStateIsIntaking()) {
                mLED.setState(Color.WHITE, mInventory.getBallCount(), false);
            } else {
                if(mSuperstructure.getSystemState() == SystemState.ENABLE_FLYWHEEL) {
                    mLED.setState(Color.BLUE, 5, false);
                } else {
                    if(mSuperstructure.getSystemState() == SystemState.AIM_LIGHTLIGHT &&
                        mSuperstructure.getAtRPM()
                    ) {
                        if(Math.abs(mDrive.getAutoSteerError()) <= 0.00827266) {
                            mLED.setState(Color.GREEN, 5, false);
                        } else {
                            mLED.setState(Color.BLUE, 5, false);
                        }
                    }
                }
            }
        }
        */
    }

    Timer testTimer = new Timer();
    LatchedBoolean testLatch1 = new LatchedBoolean();
    LatchedBoolean testLatch2 = new LatchedBoolean();
    boolean indexing = false;
    int count = 0;
    @Override
    public void testPeriodic() {
        mSuperstructure.setDisabled(true);
        mDrive.setDisabled(false);

        mDrive.setTeleOpInputs(
            mHMI.getThrottle(),
            mHMI.getStrafe(),
            mHMI.getSteer(),
            mHMI.getDriver().getBButton(),
            mHMI.getDriver().getRightTriggerBoolean(),
            mHMI.getDriver().getAButton()
        );

        //mShooter.setTargetRPM(9000);
        //mFeeder.setDemand(12);
        //mIntake.setIntake(8);
        //mIntake.setIntakeDeploy(false);

        mIntake.setIntakeDeploy(true);
        mIntake.setIntake(10);

        var trans = new Translation2d(mHMI.getThrottle(), mHMI.getStrafe());
        SmartDashboard.putNumber("Joystick Norm", trans.norm());

        /*
        mIntake.setIntake(10);

        var a = testLatch1.update(mInventory.getSensorValues()[0]);
        var b = testLatch2.update(!mInventory.getSensorValues()[0]);
        if(a) {
            count++;
            if(count < 3) indexing = true;
        }
        if(b) {
            testTimer.reset();
        }
        if((!mInventory.getSensorValues()[0] && testTimer.get() >= 0.100) || count >= 3) {
            indexing = false;
        }

        if(mHMI.getDriver().getBButton()) {
            mIntake.setIndexer(-3);
        } else if(indexing && mHMI.getDriver().getXButton()) {
            mIntake.setIndexer(3);
        } else {
            mIntake.setIndexer(0);
        }
        */


        mIntake.setIntakeDeploy(true);
        mIntake.setIntake(10);
        if(mHMI.getDriver().getAButton()) {
            mIntake.setIndexer(8);
        } else {
            mIntake.setIndexer(0);
        }
        mFeeder.setDemand(12);
        mShooter.setDemand(9.4);

        var g = mHMI.getDriver();

        /*
        int index = 0;
        if(g.getXButton()) {
            index = 3;
        } else if(g.getYButton()) {
            index = 0;
        } else if(g.getAButton()) {
            index = 2;
        } else if(g.getBButton()) {
            index = 1;
        }

        for(int i = 0; i < 4; i++) {
            if(i == index) {
                mDrive.getSwerveModules()[i].setSteerVoltage(g.getLeftStickX() * 4);
            } else {
                mDrive.getSwerveModules()[i].setSteerVoltage(0);
            }
        }
        */

        //mDrive.getSwerveModules()[0].setVoltage(12);
        //mDrive.getSwerveModules()[0].setVelocity(80);

        //mDrive.getSwerveModules()[1].setSteerVoltage(5);
        //mDrive.getSwerveModules()[3].setSteerVoltage(5);
        //mDrive.getSwerveModules()[3].setAngle(g.getLeftStickX() * 180);
    }
}