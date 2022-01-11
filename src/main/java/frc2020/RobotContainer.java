package frc2020;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc2020.hmi.HMI;
import frc2020.subsystems.*;
import frc2020.statemachines.*;
import frc2020.statemachines.SuperstructureStateMachine.SystemState;
import lib.subsystems.Subsystem;
import lib.util.LatchedBoolean;
import lib.wpilib.IRobotContainer;
import frc2020.paths.TrajectoryRegistry;

public class RobotContainer implements IRobotContainer {

    private final Drive mDrive = Drive.getInstance();
    
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    
    private final Intake mIntake = Intake.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Feeder mFeeder = Feeder.getInstance();

    private final Inventory mInventory = Inventory.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final Limelight mLimelight = Limelight.getInstance();
    private final LED mLED = LED.getInstance();
    private final Pixy mPixy = Pixy.getInstance();

    private final TrajectoryRegistry mTrajectoryRegistry = TrajectoryRegistry.getInstance();
    private final HMI mHMI = HMI.getInstance();
    
    private boolean mCoastDrive = true;

    public Subsystem[] getSubsystems() {
        var subystems = new Subsystem[] {
            mRobotStateEstimator,
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
        };
        return subystems;
    }

    /**
     * Robot-wide initialization code should go here.
     */
    public void robotInit() {
        mTrajectoryRegistry.load("Slalom", "Barrel", "Bounce1", "RedA", "RedB");

        if(!SmartDashboard.containsKey("Disable Shooter")) {
            SmartDashboard.putBoolean("Disable Shooter", false);
        }

        //CameraServer.getInstance().startAutomaticCapture();
    }

    /**
     * Initialization code for disabled mode should go here.
     */
    public void disabledInit() {
        mDrive.setBraked(true);
        mHood.forceDisable();

        SmartDashboard.putBoolean("autoInit", false);
    }

    /**
     * Initialization code for autonomous mode should go here.
     */
    public void autonomousInit() {

    }

    /**
     * Initialization code for teleop mode should go here.
     */
    public void teleopInit() {

    }

    /**
     * Initialization code for test mode should go here.
     */
    public void testInit() {

    }

    /**
     * Initalization code for any mode (other than disabled) should go here.
     */
    public void enabledInit() {
        SmartDashboard.putBoolean("autoInit", true);

        mDrive.setBraked(true);

        mSuperstructure.setDisabled(false);
        mDrive.setDisabled(false);

        mCoastDrive = false;
    }

    /**
     * Periodic code for all robot modes should go here.
     */
    public void robotPeriodic() {
        
    }

    /**
     * Periodic code for disabled mode should go here.
     */
    public void disabledPeriodic() {
        //Reset Field to Vehicle
        mRobotStateEstimator.resetOdometry();

        mSuperstructure.resetStateMachine();

        intakeOn = false;
        aimManual = false;
        enableFlywheel = false;

        boolean disableShooter = SmartDashboard.getBoolean("Disable Shooter", false);
        mSuperstructure.setDisableShooter(disableShooter);

        if(mDrive.getLinearVelocity() < Units.inchesToMeters(5)) {
            mCoastDrive = true;
        }
        mDrive.setBraked(!mCoastDrive);
        for(SwerveModule swerveModule : mDrive.getSwerveModules()) {
            swerveModule.resetOffset();
        }

        mHood.forceDisable();
    }

    /**
     * Periodic code for autonomous mode should go here.
     */
    public void autonomousPeriodic() {
        mDrive.setBraked(true);
    }

    /**
     * Periodic code for teleop mode should go here.
     */
    public void teleopPeriodic() {
        manualControl();
    }

    /**
     * Periodic code for test mode should go here.
     */
    public void testPeriodic() {

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
            false,
            mHMI.getDriver().getAButton(),
            mHMI.getDriver().getYButton() || mSuperstructure.getSystemState() == SystemState.SHOOT,
            mHMI.getDriver().getLeftTriggerBoolean());
        if(mHMI.getDriver().getBButton()) mRobotStateEstimator.resetOdometry();

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
                        /*if((mHMI.getTurretManual() != 0 || aimManual) && mHMI.getAim()) {
                            mSuperstructure.setWantAimManual();
                            aimManual = true;
                        } else */{
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

        if(mHMI.getDriver().getPOVData().getDown()) {
            mSuperstructure.setIntakeDeployOverride(true);
        } else {
            mSuperstructure.stopIntakeDeployOverride();
        }
    }
}
