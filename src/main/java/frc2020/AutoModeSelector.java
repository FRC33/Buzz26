package frc2020;

import frc2020.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        START
    }

    enum DesiredMode {
        DO_NOTHING,
        BARREL_RACING,
        SLALOM,
        BOUNCE,
        GALACTIC_SEARCH,
        TEST_PATH,
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Start", StartingPosition.START);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        // AUTO MODES GO HERE
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Barrel Racing", DesiredMode.BARREL_RACING);
        mModeChooser.addOption("Slalom", DesiredMode.SLALOM);
        mModeChooser.addOption("Bounce", DesiredMode.BOUNCE);
        mModeChooser.addOption("Galactic Search", DesiredMode.GALACTIC_SEARCH);
        mModeChooser.addOption("Test Path", DesiredMode.TEST_PATH);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case BARREL_RACING:
                return Optional.of(new BarrelRacingMode());
            case SLALOM:
                return Optional.of(new SlalomMode());
            case BOUNCE :
                return Optional.of(new BounceMode());
            case GALACTIC_SEARCH:
                return Optional.of(new GalacticSearchMode());
            case TEST_PATH:
                return Optional.of(new PathTestMode());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
}