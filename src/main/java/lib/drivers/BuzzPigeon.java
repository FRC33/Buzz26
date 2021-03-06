package lib.drivers;

import com.ctre.phoenix.sensors.PigeonIMU;

import lib.geometry.Rotation2d;

public class BuzzPigeon extends PigeonIMU {
    private double mZeroYaw;

    public BuzzPigeon() {
        this(0);
    }
    
    public BuzzPigeon(int id) {
        super(id);
    }

    public double getRawYaw() {
        double[] ypr_deg = {0, 0, 0};
        getYawPitchRoll(ypr_deg);
        return ypr_deg[0];
    }

    public double getRawYawZeroed() {
        return getRawYaw() - mZeroYaw;
    }

    public void reset() {
        mZeroYaw = getRawYaw(); 
    }
}