package frc2020.subsystems;

import static frc2020.Constants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc2020.PatternGenerator;
import frc2020.states.LEDState;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;

public class LED extends Subsystem {
    private static LED mInstance;

    private LEDState mLedState = new LEDState();

    private AddressableLED mAddressableLED;
    private AddressableLEDBuffer mAddressableLEDBuffer;

    private PatternGenerator mPatternGenerator = new PatternGenerator(30);

    public synchronized static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }

        return mInstance;
    }

    private LED() {
        mAddressableLED = new AddressableLED(kLEDId);
        mAddressableLEDBuffer = new AddressableLEDBuffer(30);

        mAddressableLED.setLength(30);
    }

    @Override
    public void registerEnabledLoops(final ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(final double timestamp) {
                synchronized (LED.this) {
                    mAddressableLED.start();
                    stop();
                }
            }

            @Override
            public void onLoop(final double timestamp) {
                synchronized (LED.this) {
                    mPatternGenerator.setRotating(new Color(0.4, 0.4, 0), 3, 6, 0.03);
                    mPatternGenerator.updateBuffer(mAddressableLEDBuffer);
                    mAddressableLED.setData(mAddressableLEDBuffer);
                }
            }

            @Override
            public void onStop(final double timestamp) {
                stop();
            }
        });
    }

    public void setState(LEDState ledState) {
        mLedState = ledState;
    }

    @Override
    public synchronized void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {

    }
}