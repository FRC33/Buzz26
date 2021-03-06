package lib.drivers;

import edu.wpi.first.wpilibj.PWMSpeedController;

public class SmartServo extends PWMSpeedController {
  public SmartServo(final int channel, boolean invert) {
    super(channel);

    setBounds(2.5, 1.51, 1.49, 1.48, 0.51);
    setPeriodMultiplier(PeriodMultiplier.k1X);
    setSpeed(0.0);
    setZeroLatch();

    setInverted(invert);
  }
}
