package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.ArmExtension;


@Config
public class CompoundArm {
    private ArmExtension armExtension;
    double armExtensionTarget;
    private LinearOpMode opMode;
    private RobotHardware robotHardware;

    // Initializes this.
    public CompoundArm(LinearOpMode opMode, RobotHardware robotHardware) throws InterruptedException {

        // Remember the op mode.
        this.opMode = opMode;

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Initialize the FTC dashboard.
        FtcDashboard.getInstance();

        armExtension = new ArmExtension(robotHardware);
    }

    // Updates this.
    public void update() throws InterruptedException {

        // Verify input exists.
        if (robotHardware.slideServo == null) {
            throw new InterruptedException("The slide servo is missing.");
        }
        if (robotHardware.slideEncoder == null) {
            throw new InterruptedException("The slide servo encoder is missing.");
        }

        // Update the slide.
        //////////////////////////////////////////////////////////////////////

        // Get the slide servo.
        CRServo slideServo = robotHardware.slideServo;

        // Get the slide encoder.
        AnalogInput slideEncoder = robotHardware.slideEncoder;

        // Get the slide's power.
        double slidePower = slideServo.getPower();

        // Get the slide's voltage.
        double slideVoltage = slideEncoder.getVoltage();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display slide telemetry.
        telemetry.addData("Slide", "====================");
        telemetry.addData("- Power", slidePower);
        telemetry.addData("- Voltage", slideVoltage);

        // Update the slider.
        //////////////////////////////////////////////////////////////////////

        double currentExtensionPos = armExtension.getPosition(false);
        double armExtensionDistance = armExtensionTarget - currentExtensionPos;
        armExtension.setPower(armExtensionDistance / 10);
    }

    public void extendArm(double armExtensionTarget) {
        this.armExtensionTarget = armExtensionTarget;
    }

    public ArmExtension getArmExtension() {
        return armExtension;
    }

//    // Retracts the slide
//    public void retractSlide() {
//        CRServo slideServo = robotHardware.slideServo;
//        slideServo.setPower(RETRACT_SLIDE_POWER);
//    }
//
//    // Extends the slide.
//    public void extendSlide() {
//        CRServo slideServo = robotHardware.slideServo;
//        slideServo.setPower(EXTEND_SLIDE_POWER);
//    }
//
//    // Stops the slide.
//    public void stopSlide() {
//        CRServo slideServo = robotHardware.slideServo;
//        slideServo.setPower(0);
//    }

}