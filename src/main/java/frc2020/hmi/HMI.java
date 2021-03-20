package frc2020.hmi;

import frc2020.Constants;
import frc2020.ShootingLocation.Location;
import lib.drivers.BuzzXboxController;
import lib.drivers.BuzzXboxController.POV;

/**
 * Maps raw controller inputs to specific driver and operator functions
 */
public class HMI {
    private static HMI mInstance;

    private BuzzXboxController driver;
    private BuzzXboxController operator;

    public synchronized static HMI getInstance() {
        if (mInstance == null) {
            mInstance = new HMI();
        }

        return mInstance;
    }

    public HMI() {
        driver = new BuzzXboxController(0);
        operator = new BuzzXboxController(1);
    }

    public BuzzXboxController getDriver() {
        return driver;
    }

    public BuzzXboxController getOperator() {
        return operator;
    }

    //region Driving
    public double getThrottle() {
        return -driver.getLeftStickY();
    }

    public double getStrafe() {
        return driver.getLeftStickX();
    } 

    public double getSteer() {
        return driver.getRightStickX();
    }
    //endregion

    //region Intaking
    public boolean getToggleCollect() {
        return driver.getLeftBumper();
    }

    public boolean getBlow() {
        return driver.getXButton();
    }

    public boolean getIntakeIn() {
        return operator.getLeftBumper();
    }

    public boolean getIntakeOut() {
        return operator.getRightBumper();
    }
    //endregion

    //region Shooting
    public Location getShootingLocation() {
        POV pov = operator.getPOVData();
        if(pov.getTop()) return Location.GREEN;
        if(pov.getRight()) return Location.YELLOW;
        if(pov.getDown()) return Location.BLUE;
        if(pov.getLeft()) return Location.RED;
        return Location.NONE;
    }

    public boolean getAltitudeInc() {
        return operator.getYButton();
    }

    public boolean getAltitudeDec() {
        return operator.getAButton();
    }

    public boolean getAim() {
        return driver.getLeftTriggerBoolean();
    }

    public boolean getShoot() {
        return driver.getRightTriggerBoolean();
    }

    public boolean getClearShot() {
        return operator.getRightTriggerBoolean();
    }
    //endregion

    //region Climbing
    public boolean getMastUp() {
        return driver.getYButton();
    }

    public boolean getMastDown() {
        return driver.getRightSmallButton();
    }

    public boolean getWinchIn() {
        return driver.getLeftSmallButton();
    }
    
    public boolean getWinchOut() {
        return driver.getPOVData().getDown();
    }
    //endregion

    //region Manuals
    public boolean getBrushForward() {
        return operator.getBButton();
    }
    
    public boolean getBrushBackward() {
        return operator.getXButton();
    }

    public double getHoodManual() {
        return -operator.getLeftStickY();
    }

    public double getTurretManual() {
        return operator.getRightStickX();
    }
    //endregion
}