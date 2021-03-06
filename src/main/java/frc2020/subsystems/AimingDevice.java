package frc2020.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;

/**
Currently just combines the turret and the hood. Exists in case advanced interactions between the turret and hood
(i.e path planning) are added.
*/
public class AimingDevice extends Subsystem {
    private static AimingDevice mInstance;

    private static Turret mTurret = Turret.getInstance();
    private static Hood mHood = Hood.getInstance();

    private double mTurretSetpoint;
    private double mHoodSetpoint;

    private boolean disabled = false;

    public synchronized static AimingDevice getInstance() {
        if (mInstance == null) {
            mInstance = new AimingDevice();
        }

        return mInstance;
    }

    private AimingDevice() {

    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (AimingDevice.this) { }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (AimingDevice.this) {
                    if(!disabled) {
                        mTurret.setAngle(mTurretSetpoint);
                        mHood.setAngle(mHoodSetpoint);
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                synchronized (AimingDevice.this) {
                    stop();
                }
            }
        };

        mEnabledLooper.register(mLoop);
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    public void setTurretSetpoint(double angle) {
        mTurretSetpoint = angle;
    }

    public void setHoodSetpoint(double angle) {
        mHoodSetpoint = angle;
    }

    public double getTurretAngle() {
        return mTurret.getAngle();
    }

    public double getHoodAngle() {
        return mHood.getAngle();
    }

    @Override
    public synchronized void stop() {
        mHood.stop();
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() { }
}