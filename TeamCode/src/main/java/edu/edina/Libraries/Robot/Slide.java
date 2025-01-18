package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.Actions.LinearMotionAction;
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
        LinearMotionAction lma = new LinearMotionAction(aem, target);
        return lma;
    }
}