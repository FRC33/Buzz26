package frc2020.auto.actions;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc2020.subsystems.Drive;
import frc2020.subsystems.Limelight;
import frc2020.subsystems.Superstructure;
import lib.util.Util;

public class AimAction implements Action {

    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Limelight mLimelight = Limelight.getInstance();

    private double mAngleEpsilon;
    private PIDController mAutoAimController = new PIDController(0.2, 0, 0);

    public AimAction() {
        this(0.3);
    }

    public AimAction(double angleEpsilon) {
        mAngleEpsilon = angleEpsilon;
    }

    @Override
    public void start() {
        mAutoAimController.setSetpoint(0);
        mAutoAimController.calculate(mLimelight.getXAngle());

        mSuperstructure.setWantAimLimelight();
    }

    @Override
    public void update() {
        mAutoAimController.setSetpoint(0);
        double adjust = mAutoAimController.calculate(mLimelight.getXAngle());
        
        double limit = 1;
        if(adjust > limit) {
            adjust = limit;
        } else if(adjust < -limit) {
            adjust = -limit;
        }

        System.out.println(adjust);

        mDrive.setFieldRelativeChassisSpeeds(0, adjust, 0);
    }

    @Override
    public boolean isFinished() {
        return Util.epsilonEquals(mAutoAimController.getPositionError(), 0, mAngleEpsilon) && mLimelight.getValid();
    }

    @Override
    public void done() {
        
    }

}
