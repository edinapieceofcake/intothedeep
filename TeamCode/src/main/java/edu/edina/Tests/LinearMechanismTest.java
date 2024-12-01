package edu.edina.Tests;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import edu.edina.Libraries.LinearMotion.AxialDriveMechanism;
import edu.edina.Libraries.LinearMotion.ILinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.LinearMotionController;
import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class LinearMechanismTest extends LinearOpMode {
    private ILinearMechanism linearMech;
    private LinearMechanismSettings settings;
    private static double accel = 0;
    private static double dist = 0;

    public void runOpMode() throws InterruptedException {
        linearMech = new AxialDriveMechanism(new RobotHardware(this));
        settings = linearMech.getSettings();
        double accelMax = 1.0 / settings.ka;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up)
                accel += 0.001;
            else if (gamepad1.dpad_down)
                accel -= 0.001;

            if (gamepad1.dpad_left)
                dist -= 0.01;
            else if (gamepad1.dpad_right)
                dist += 0.01;

            if (accel > accelMax)
                accel = accelMax;
            if (accel < -accelMax)
                accel = -accelMax;

            telemetry.addData("acceleration to test (a button)", accel);
            telemetry.addData("distance to test (b button)", dist);
            telemetry.update();

            if (gamepad1.a)
                runAccelTest(accel);

            if (gamepad1.b)
                runDistTest(dist);

            linearMech.setPower(0);
        }
    }

    public void runAccelTest(double accel) {
        ArrayList<Double> positions = new ArrayList<>();
        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive()) {
            DualNum<Time> u = linearMech.getPositionAndVelocity(false);

            double position = u.get(0);
            positions.add(position);
            double velocity = u.get(1);

            double power = settings.ks + settings.kv * velocity + settings.ka * accel;

            if (Math.abs(power) > 1)
                break;

            linearMech.setPower(power);

            if (!gamepad1.a)
                break;

            telemetry.addData("position", position);
            telemetry.addData("velocity", velocity);
            telemetry.addData("power", power);
            telemetry.update();
        }

        linearMech.setPower(0);

        double firstPos = positions.get(0);
        double lastPos = positions.get(positions.size() - 1);
        double t = et.seconds();

        while (opModeIsActive()) {
            telemetry.addData("time", et);
            telemetry.addData("distance traveled", lastPos - firstPos);
            telemetry.addData("theoretical distance", 0.5 * accel * t * t);
            telemetry.update();
        }
    }

    public void runDistTest(double targetDist) {
        LinearMotionController con = new LinearMotionController(linearMech);

        double startX = linearMech.getPosition(false);
        con.setTarget(startX + targetDist);

        boolean done = false;
        double actualDist = 0;

        while (opModeIsActive()) {
            if (gamepad1.b)
                done = con.run();
            else
                linearMech.setPower(0);

            DualNum<Time> u = con.lastPositionAndVelocity();
            actualDist = u.get(0) - startX;

            telemetry.addData("targetDist", "%.2f", targetDist);
            telemetry.addData("actualDist", "%.2f", actualDist);
            telemetry.addData("velocity", "%.2f", u.get(1));
            telemetry.addData("nominal accel", "%.2f", settings.nominalAccel);
            if (done)
                telemetry.addLine("done");

            telemetry.update();
        }
    }
}