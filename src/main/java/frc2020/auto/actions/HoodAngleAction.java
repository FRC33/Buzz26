package frc2020.auto.actions;

import frc2020.subsystems.Hood;
import frc2020.subsystems.Superstructure;
import lib.util.Util;

public class HoodAngleAction implements Action {

    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Hood mHood = Hood.getInstance();

    private double mTargetHoodAngle;
    private double mEpsilon;

    public HoodAngleAction(double targetHoodAngle, double epsilon) {
        mTargetHoodAngle = targetHoodAngle;
        mEpsilon = epsilon;
    }

    @Override
    public void start() {
        mSuperstructure.setHoodAngleOverride(mTargetHoodAngle);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Util.epsilonEquals(mHood.getAngle(), mTargetHoodAngle, mEpsilon);
    }

    @Override
    public void done() {}
}