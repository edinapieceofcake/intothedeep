package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        // While the op mode is active...
        while (!isStopRequested()) {
            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If the user has not selected an alliance...
            if (redAlliance == null) {
                telemetry.addData("Alliance", "X = blue, B = red");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    redAlliance = false;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    redAlliance = true;
                }
            }

            // Otherwise (if the user finished making menu selections)...
            else {

                // If the user has not selected a side...
                if (basketSide == null) {
                    telemetry.addData("Side", "X = basket, B = chamber");
                    telemetry.update();
                    if (currentGamepad.x && !previousGamepad.x) {
                        basketSide = false;
                    }
                    if (currentGamepad.b && !previousGamepad.b) {
                        basketSide = true;
                    }
                }

                // Otherwise (if the user finished making menu selections)...
                else {

                    // Stop prompting the user for inputs.
                    break;

                }

            }

        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.a && !previousGamepad.a) {
                compoundArm.toggleClaw();
            }

//            if (currentGamepad.x && !previousGamepad.x) {
//                compoundArm.toggleWrist();
//            }

            driveTrain.update();

            compoundArm.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
