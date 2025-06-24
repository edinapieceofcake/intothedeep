package edu.edina.Libraries.Actions;

public class ControllingActionManager {
    private ControllingAction currentAction;

    public void cancelControllingAction() {
        if (currentAction != null)
            currentAction.cancel();

        currentAction = null;
    }

    public void updateControllingAction(ControllingAction action, boolean running) {
        if (running) {
            if (currentAction != null)
                if (currentAction != action)
                    currentAction.cancel();

            currentAction = action;
        } else {
            if (currentAction == action)
                currentAction = null;
        }
    }
}
