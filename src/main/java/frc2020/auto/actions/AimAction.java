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

    private PIDController autoAimController = new PIDController(0.2, 0, 0);

    @Override
    public void start() {
        autoAimController.setSetpoint(0);
        autoAimController.calculate(mLimelight.getXAngle());
    }

    @Override
    public void update() {
        autoAimController.setSetpoint(0);
        double adjust = autoAimController.calculate(mLimelight.getXAngle());
        
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
        return Util.epsilonEquals(autoAimController.getPositionError(), 0, 0.3) && mLimelight.getValid();
    }

    @Override
    public void done() {
        
    }

}
