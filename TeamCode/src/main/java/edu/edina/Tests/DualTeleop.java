package edu.edina.Tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Actions.KillSwitchAction;
import edu.edina.Libraries.Actions.OdometryUpdater;
import edu.edina.Libraries.Actions.RaiseLift;
import edu.edina.Libraries.Actions.SpecimenPark;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.DualClaw;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.Slide;
import edu.edina.Libraries.Robot.Swivel;
import edu.edina.Libraries.Robot.WaitForTime;
import edu.edina.Libraries.Robot.Wrist;

@TeleOp
public class DualTeleop extends LinearOpMode {
    private Swivel swivel;
    private Wrist wrist;
    private DualClaw claw;
    private Arm arm;

    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        swivel = hw.getSwivel();
        Slide slide = new Slide(hw);
        wrist = hw.getWrist();
        claw = hw.getDualClaw();
        arm = hw.getArm();

        hw.addAction(new OdometryUpdater(hw));

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (gamepad1.dpad_up) {
                hw.clearActions();
                hw.addAction(slide.extendToAmount(12));
            }
            if (gamepad1.dpad_down) {
                hw.clearActions();
                hw.addAction(slide.extendToAmount(-.2));
            }

            hw.drivetrain.update();

            hw.runActions();
        }
    }

    private Action scoreSample() {
        return new SequentialAction(
                swivel.turnToPerpendicular(),
                new WaitForTime(500),
                claw.openBoth()
        );
    }

    private Action scoreSpecimen() {
        return new ParallelAction(
                claw.closeSmall(),
                wrist.scorePosition(),
                swivel.turnToScore()
        );
    }

    private Action returnToWall() {
        return new ParallelAction(
                claw.openBoth(),
                wrist.wallPosition(),
                swivel.turnToWall()
        );
    }

    private Action submersibleGrab() {
        return new ParallelAction(
                claw.openBoth(),
                wrist.submersibleGrab(),
                swivel.turnToWall()
        );
    }
}