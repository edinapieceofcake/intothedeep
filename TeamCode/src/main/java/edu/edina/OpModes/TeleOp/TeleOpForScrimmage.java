package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.Robot.GamePadClick;

@Config
@TeleOp
public class TeleOpForScrimmage extends LinearOpMode {
    private DriveTrain driveTrain;
    private CompoundArm compoundArm;
    private ElapsedTime runtime = new ElapsedTime();

    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();
    private Boolean redAlliance;
    private Boolean basketSide;

    public void runOpMode() throws InterruptedException {
        // Get the robot drive train.
        driveTrain = new DriveTrain(this);

        // Get the robot compound arm
        compoundArm = new CompoundArm(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        GamePadClick click1=new GamePadClick(gamepad1);
        while (opModeIsActive()) {
            click1.read();

            if (click1.a) {
                compoundArm.toggleClaw();
            }

            if (click1.x) {
                compoundArm.toggleWrist();
            }

            if (click1.dpad_down) {
                compoundArm.setArmPosition(1);
            }

            if (click1.dpad_left) {
                compoundArm.setArmPosition(2);
            }

            if (click1.dpad_up) {
                compoundArm.setArmPosition(3);
            }

            if (click1.dpad_right) {
                compoundArm.setArmPosition(4);
            }

            if (click1.y) {
                compoundArm.extendArm(6);
            }

            if (click1.left_bumper) {

            }

            if (click1.left_bumper && gamepad1.b) {

            }

            if (click1.right_bumper) {

            }

            if (click1.right_bumper && gamepad1.b) {

            }


            driveTrain.update();

            compoundArm.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("extension pos", compoundArm.getArmExtension().getPosition(false));
            telemetry.update();
        }
    }
}
