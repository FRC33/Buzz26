package frc2020.subsystems;

import static frc2020.Constants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.statemachines.ClimberStateMachine;
import lib.drivers.BuzzDigitalInput;
import lib.drivers.BuzzTalonSRX;
import lib.drivers.TalonSRXFactory;
import lib.loops.ILooper;
import lib.loops.Loop;
import lib.subsystems.Subsystem;

public class Climber extends Subsystem {
    private static Climber mInstance;

    private ClimberStateMachine mStateMachine = new ClimberStateMachine();
    private ClimberStateMachine.WantedAction mWantedAction = ClimberStateMachine.WantedAction.STOP;

    private BuzzTalonSRX mMast;
    private BuzzTalonSRX mWinch; 
    private BuzzDigitalInput mMastSwitch;
    
    private boolean mPartyMode;

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }

        return mInstance;
    }

    private Climber() {
        mPeriodicIO = new PeriodicIO();

        //Initalize subsystem devices
        mMast = TalonSRXFactory.createDefaultTalon(kMastId);
        mWinch = TalonSRXFactory.createDefaultTalon(kWinchId);

        mMastSwitch = new BuzzDigitalInput(kMastSwitch);
        mMastSwitch.invert(true);
        mMastSwitch.enableDebounce(true);
    }

    private final PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public boolean mastDown;
        public boolean mastStalled;

        // OUTPUTS
        public double mastDemand;
        public double winchDemand;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        // Read inputs
        mPeriodicIO.mastDown = mMastSwitch.get();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set output
        mMast.setDemandVoltage(mPeriodicIO.mastDemand);
        mWinch.setDemandVoltage(mPeriodicIO.winchDemand);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Climber.this) { }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Climber.this) {
                    ClimberStateMachine.ClimberState newState = mStateMachine.update(timestamp, mWantedAction);
                    // Update actuators from desired state
                    mPeriodicIO.mastDemand = newState.mastVoltage;
                    mPeriodicIO.winchDemand = newState.winchVoltage;
                    mPartyMode = newState.partyMode;
                }
            }

            @Override
            public void onStop(double timestamp) {
                synchronized (Climber.this) {
                    mWantedAction = ClimberStateMachine.WantedAction.STOP;
                    stop();
                }
            }
        };

        mEnabledLooper.register(mLoop);
    }

    public boolean isMastDown() {
        return mPeriodicIO.mastDown;
    }

    public synchronized ClimberStateMachine.SystemState getSystemState() {
        return mStateMachine.getSystemState();
    }

    public boolean getPartyMode() {
        return mPartyMode;
    }

    public synchronized void resetStateMachine() {
        mStateMachine.reset();
    }
    
    public synchronized void setWantedAction(ClimberStateMachine.WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    @Override
    public void zeroSensors() {
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
        SmartDashboard.putBoolean("Mast Down", mPeriodicIO.mastDown);
        SmartDashboard.putNumber("Mast Demand", mPeriodicIO.mastDemand);
        SmartDashboard.putNumber("Winch Demand", mPeriodicIO.winchDemand);
        SmartDashboard.putString("Climber System State", mStateMachine.getSystemState().toString());
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}