package edu.edina.Libraries.MotionControl;

import edu.edina.Libraries.Robot.MotionControlSettings;

public interface IMotionControlLinearMechanism extends ILinearMechanism {
    double getAccelCalDistance();

    MotionControlSettings getMotionSettings();

    void setCurrentAction(ICancelableAction action);
}

