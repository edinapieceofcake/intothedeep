package edu.edina.Libraries.Actions;

import com.acmerobotics.roadrunner.Action;

import java.util.List;

public class ActionList {
    public static boolean manualDrive(List<Action> actions) {
        for (Action a : actions) {
            if (a.getClass().getAnnotation(AutoDrive.class) != null)
                return false;
        }

        return true;
    }
}
