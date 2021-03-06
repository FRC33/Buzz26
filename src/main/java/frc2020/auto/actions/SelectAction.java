package frc2020.auto.actions;

import java.util.function.Supplier;

/** When this action starts, it will select one of two actions to run based on a condition supplied by a boolean function */
public class SelectAction implements Action {

    private Supplier<Boolean> mCondition;
    private Action mTrueAction;
    private Action mFalseAction;

    private Action mSelectedAction;

    public SelectAction(Supplier<Boolean> condition, Action trueAction, Action falseAction) {
        mCondition = condition;
        mTrueAction = trueAction;
        mFalseAction = falseAction;
    }

    @Override
    public void start() {
        mSelectedAction = mCondition.get() ? mTrueAction : mFalseAction;
        mSelectedAction.start();
    }

    @Override
    public void update() {
        mSelectedAction.update();
    }

    @Override
    public boolean isFinished() {
        return mSelectedAction.isFinished();
    }

    @Override
    public void done() {
        mSelectedAction.done();
    }
    
}
