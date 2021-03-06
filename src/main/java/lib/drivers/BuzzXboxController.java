package lib.drivers;

import edu.wpi.first.wpilibj.XboxController;

public class BuzzXboxController extends XboxController {
    public BuzzXboxController(int port) {
        super(port);
    }

    public double getLeftStickX() {
        return deadband(getX(Hand.kLeft));
    }
    
    public double getLeftStickY() {
        return deadband(getY(Hand.kLeft));
    }

    public double getRightStickX() {
        return deadband(getX(Hand.kRight));
    }
    
    public double getRightStickY() {
        return deadband(getY(Hand.kRight));
    }

    public boolean getLeftTriggerBoolean() {
        return getTriggerAxis(Hand.kLeft) > 0.2;
    }

    public boolean getRightTriggerBoolean() {
        return getTriggerAxis(Hand.kRight) > 0.2;
    }

    public boolean getLeftBumper() {
        return getBumper(Hand.kLeft);
    }
    
    public boolean getRightBumper() {
        return getBumper(Hand.kRight);
    }

    public boolean getLeftSmallButton() {
        return getBackButton();
    }

    public boolean getRightSmallButton() {
        return getStartButton();
    }

    //region POV
    public static class POV {
        private boolean mTop;
        private boolean mRight;
        private boolean mDown;
        private boolean mLeft;

        public POV(boolean top, boolean right, boolean down, boolean left) {
            mTop = top;
            mRight = right;
            mDown = down;
            mLeft = left;
        }

        public POV() {

        }

        public boolean getTop() {return mTop;}
        public boolean getRight() {return mRight;}
        public boolean getDown() {return mDown;}
        public boolean getLeft() {return mLeft;}
    }

    private static final POV[] povValues = {
        new POV(true, false, false, false),
        new POV(true, true, false, false),
        new POV(false, true, false, false),
        new POV(false, true, true, false),
        new POV(false, false, true, false),
        new POV(false, false, true, true),
        new POV(false, false, false, true),
        new POV(true, false, false, true)
    };

    public POV getPOVData() {
        int degrees = super.getPOV();
        return degrees != -1 ? povValues[degrees / 45] : new POV();
    }
    //endregion

    private static double deadband(double input) {
        return deadband(input, 0.08);
    }

    private static double deadband(double input, double deadbandValue) {
        return (input > -deadbandValue && input < deadbandValue) ? 0 : input;
    }
}