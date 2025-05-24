package edu.edina.Libraries.MotionControl;

import com.acmerobotics.roadrunner.Action;

public interface ICancelableAction extends Action {
    void cancel();
}
