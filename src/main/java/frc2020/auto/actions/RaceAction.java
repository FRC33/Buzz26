package frc2020.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Composite action, running all sub-actions at the same time All actions are started then updated one action
 * reports being done.
 */
public class RaceAction implements Action {
    private final ArrayList<Action> mActions;

    public RaceAction(List<Action> actions) {
        mActions = new ArrayList<>(actions);
    }

    public RaceAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public void start() {
        mActions.forEach(Action::start);
    }

    @Override
    public void update() {
        mActions.forEach(Action::update);
    }

    @Override
    public boolean isFinished() {
        for (Action action : mActions) {
            if (action.isFinished()) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void done() {
        mActions.forEach(Action::done);
    }
}