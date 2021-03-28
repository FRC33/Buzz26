package lib.drivers;

import edu.wpi.first.wpilibj.XboxController;

public class BuzzXboxController extends XboxController {
    public BuzzXboxController(int port) {
        super(port);
    }

    public double getLeftStickX() {
        return getX(Hand.kLeft);
    }
    
    public double getLeftStickY() {
        return getY(Hand.kLeft);
    }

    public double getRightStickX() {
        return getX(Hand.kRight);
    }
    
    public double getRightStickY() {
        return getY(Hand.kRight);
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

    /**
     * Uses default deadband cutoff of 0.08
     */
    public static double deadband(double input) {
        return deadband(input, 0.08);
    }

    public static double deadband(double input, double deadbandCutoff) {
        return (input > -deadbandCutoff && input < deadbandCutoff) ? 0 : input;
    }

    public static double cubic(double x, double weight) {
        return weight * x * x * x  + (1.0 - weight) * x;
    }
    
    /**
     * Uses default deadband cutoff of 0.08
     */
    public static double joystickCubicScaledDeadband(double x, double weight) {
        return joystickCubicScaledDeadband(x, 0.08, weight);
    }

    public static double joystickCubicScaledDeadband(double x, double deadbandCutoff, double weight) {
        if(Math.abs(x) < deadbandCutoff) {
            return 0;
        } else {
            return (cubic(x, weight)- (Math.abs(x)/x)* cubic(deadbandCutoff, weight)) / (1.0 - cubic(deadbandCutoff, weight));
        }
    } 
}