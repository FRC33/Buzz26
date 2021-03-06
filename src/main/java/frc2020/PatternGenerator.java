package frc2020;

import java.util.regex.Pattern;

import edu.wpi.first.hal.simulation.AddressableLEDDataJNI;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class PatternGenerator {
    private final int mStripLength;

    private PatternMode mPatternMode = PatternMode.OFF;

    private Color mColor1 = Color.kBlack;
    private int mDashLength = 0;
    private int mDashFullLength = 0;
    private double mCycleTime = 0;

    private Timer mTimer = new Timer();
    private int mOffset = 0;

    private enum PatternMode {
        OFF, SOLID, ROTATING
    }

    public PatternGenerator(int stripLength) {
        mStripLength = stripLength;
        reset();
    }

    public void reset() {
        mTimer.reset();
        mTimer.start();
        mOffset = 0;
    }

    public void setOff() {
        mPatternMode = PatternMode.OFF;
    }

    public void setSolid(Color color) {
        mPatternMode = PatternMode.SOLID;
        mColor1 = color;
    }

    public void setRotating(Color color, int dashLength, int dashFullLength, double cycleTime) {
        mPatternMode = PatternMode.ROTATING;
        mColor1 = color;
        mDashLength = dashLength;
        mDashFullLength = dashFullLength;
        mCycleTime = cycleTime;
    }

    public void updateBuffer(AddressableLEDBuffer buffer) {
        switch(mPatternMode) {
            case SOLID:
                updateBufferSolid(buffer);
                break;
            case ROTATING:
                updateBufferRotating(buffer);
                break;
            case OFF:
                // Fall through to default
            default:
                updateBufferOff(buffer);
                break;

        }
    }

    private void updateBufferOff(AddressableLEDBuffer buffer) {
        for(int i = 0; i < mStripLength; i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }

    private void updateBufferSolid(AddressableLEDBuffer buffer) {
        for(int i = 0; i < mStripLength; i++) {
            buffer.setLED(i, mColor1);
        }
    }

    private void updateBufferRotating(AddressableLEDBuffer buffer) {
        if(mTimer.get() >= mCycleTime) {
            mOffset++;
            if(mOffset >= mDashFullLength) mOffset = 0;
            mTimer.reset();
        }

        int counter = 0;
        for(int i = 0; i < mOffset; i++) {
            buffer.setLED(i, Color.kBlack);
        }
        for(int i = mOffset; i < mStripLength; i++) {
            buffer.setLED(i, counter < mDashLength ? mColor1 : Color.kBlack);
            counter++;
            if(counter >= mDashFullLength) counter = 0;
        }
    }
}
