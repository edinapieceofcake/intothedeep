package edu.edina.Libraries.Actions;

import com.acmerobotics.roadrunner.Action;

import java.util.List;

public class ActionListOld {
    public static boolean canControlPart(String neededPart, List<Action> actions) {
        if (neededPart == null)
            return true;

        for (Action a : actions) {
            Controls[] activeControls = a.getClass().getAnnotationsByType(Controls.class);
            for (Controls active : activeControls) {
                if (neededPart.equals(active.part()))
                    return false;
            }
        }

        return true;
    }

    public static <T> boolean canAddAction(Class<T> actionClass, List<Action> actions) {
        Controls[] neededControls = actionClass.getAnnotationsByType(Controls.class);

        for (Controls needed : neededControls)
            if (!canControlPart(needed.part(), actions))
                return false;

        return true;
    }
}