package frc2020.auto.actions;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class WaitLambdaAction implements Action {

    Supplier<Boolean> mSupplier;

    public WaitLambdaAction(Supplier<Boolean> supplier) {
        mSupplier = supplier;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mSupplier.get();
    }

    @Override
    public void done() {}
}
