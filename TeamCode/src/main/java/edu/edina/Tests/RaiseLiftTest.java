package edu.edina.Tests;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;

import edu.edina.Libraries.Actions.RaiseLift;
import edu.edina.Libraries.Robot.RobotHardware;
@TeleOp
@Disabled
public class RaiseLiftTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(this);
        waitForStart();
        Action action;
        while (opModeIsActive()) {
            if (gamepad1.a){
               action = new SequentialAction(
                        new RaiseLift(robotHardware,14)
                );

            } else {
                action = new SequentialAction(
                        new RaiseLift(robotHardware,0)
                );
            }
            action.run(new TelemetryPacket());
        }
    }
}
