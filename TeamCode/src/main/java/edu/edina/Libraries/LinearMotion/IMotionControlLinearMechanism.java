package edu.edina.Libraries.LinearMotion;

import edu.edina.Libraries.Robot.MotionControlSettings;

public interface IMotionControlLinearMechanism extends ILinearMechanism {
    MotionControlSettings getMotionSettings();
}
