package edu.edina.Tests;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.RoadRunner.Localizer;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;

@TeleOp
public class Localizer_Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Localizer loc = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);
        Twist2dDual<Time> twist;

        waitForStart();
        while (opModeIsActive()) {
            twist = loc.update();
            telemetry.addData("Line", "(%.2f, %.2f)", twist.line.x.get(0), twist.line.y.get(0));
            telemetry.addData("Linear Velocity", "(%.2f, %.2f)", twist.line.x.get(1), twist.line.y.get(1));
            telemetry.addData("Angle", "%.2f",Math.toDegrees(twist.angle.get(0)));
            telemetry.addData("Angle Velocity", "%.2f",Math.toDegrees(twist.angle.get(1)));
            telemetry.update();
        }
    }
}
