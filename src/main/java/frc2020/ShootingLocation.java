package frc2020;

public class ShootingLocation {
    private static Parameters mParameters[] = {
        new Parameters(50, 0, 0, 0),
        new Parameters(33.5, 4000, 0, 0), //GREEN
        new Parameters(43.5, 5000, 0, 0), //YELLOW
        new Parameters(51.5, 5600, 0, 0), //BLUE
        new Parameters(51, 5000, 0, 0), //RED
    };

    public static class Parameters {
        private double mHoodAngle, mShooterRPM, mTurretAngle, mTargetOffset;
        
        public Parameters(double hoodAngle, double shooterRPM, double turretAngle, double targetOffset) {
            mHoodAngle = hoodAngle;
            mShooterRPM = shooterRPM;
            mTurretAngle = turretAngle;
            mTargetOffset = targetOffset;
        }

        public double getHoodAngle() { return mHoodAngle; }
        public double getShooterRPM() { return mShooterRPM; }
        public double getTurretAngle() { return mTurretAngle; }
        public double getTargetOffset() { return mTargetOffset; }
    }

    public static enum Location {
        NONE(0), GREEN(1), YELLOW(2), BLUE(3), RED(4);

        private int mIndex;

        public int getIndex() {
            return mIndex;
        }

        public double getHoodAngle() {
            return mParameters[getIndex()].getHoodAngle();
        }
    
        public double getShooterRPM() {
            return mParameters[getIndex()].getShooterRPM();
        }
    
        public double getTurretAngle() {
            return mParameters[getIndex()].getTurretAngle();
        }
    
        public double getTargetOffset() {
            return mParameters[getIndex()].getTargetOffset();
        }

        private Location(int index) {
            mIndex = index;
        }
    }
}