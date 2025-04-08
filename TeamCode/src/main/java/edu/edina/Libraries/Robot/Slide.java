package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import edu.edina.Libraries.Actions.RunToPositionAction;
import edu.edina.Libraries.LinearMotion.ArmExtensionMechanism;

@Config
public class Slide {
    private DcMotorEx motor;
    private RobotHardware hw;

    public Slide(RobotHardware hw) {
        this.hw = hw;
        motor = this.hw.getExtension();
    }

    public Action extendToAmount(double target) {
        ArmExtensionMechanism aem = new ArmExtensionMechanism(hw);
        RunToPositionAction lma = new RunToPositionAction(aem, target);
        return lma;
    }
}