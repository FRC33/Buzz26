package frc2020.auto.actions;

import java.util.function.Supplier;

public class HangingAction implements Action {

    Action mAction;
    Supplier<Boolean> mStopSupplier;

    /**
     * Will run <code>action</code> until the <code>stopSupplier</code> returns <code>true</code> (even if the <code>action</code> wants to stop)
     * @param action The action to run
     * @param stopSupplier The supplier that indicates when the action should stop
     */
    public HangingAction(Action action, Supplier<Boolean> stopSupplier) {
        mAction = action;
        mStopSupplier = stopSupplier;
    }

    @Override
    public void start() {
        mAction.start();
    }

    @Override
    public void update() {
        mAction.update();
    }

    @Override
    public boolean isFinished() {
        return mStopSupplier.get();
    }

    @Override
    public void done() {
        mAction.done();
    }
}
