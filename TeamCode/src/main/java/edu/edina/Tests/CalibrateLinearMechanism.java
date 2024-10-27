package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.Robot.LinearDrive;
import edu.edina.Libraries.Robot.LinearFunc;
import edu.edina.Libraries.Robot.LinearFuncFitter;
import edu.edina.Libraries.Robot.Speedometer;

@TeleOp
public class CalibrateLinearMechanism extends LinearOpMode {
    double powerStep = 0.02;
    double speedThres = 1;

    LinearDrive axialDrive;

    @Override
    public void runOpMode() {
        axialDrive = new LinearDrive(hardwareMap);

        waitForStart();

        telemetry.addLine("Press a to calibrate ppi");
        telemetry.addLine("Press b to calibrate Kv");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                calibratePpi();
            }
            if (gamepad1.b) {
                calibrateLinearKs();
            }
        }
    }

    private void calibratePpi() {
        while (opModeIsActive()) {
            telemetry.addLine("press up to drive forward");

            if (gamepad1.dpad_up) {
                axialDrive.setPower(0.3);
            } else {
                axialDrive.setPower(0.0);
            }

            telemetry.addData("encoder (raw)", axialDrive.getPosition(true));
            telemetry.addData("encoder (inch)", axialDrive.getPosition(false));
            telemetry.update();
        }
    }

    private void calibrateLinearKs() {
        Speedometer speedometer = new Speedometer(10);
        double power = powerStep;

        double prevRead = 0.0;
        double finalSpeed = 0.0;

        List<Double> powerList = new ArrayList<Double>();
        List<Double> speedList = new ArrayList<Double>();

        while (opModeIsActive()) {
            while (opModeIsActive()) {
                axialDrive.setPower(power);

                if (gamepad1.start) {
                    break;
                }

                speedometer.sample(axialDrive.getPosition(false) - prevRead);

                telemetry.addData("speed", speedometer.getSpeed());
                telemetry.addData("power", power);
                telemetry.update();
            }

            axialDrive.setPower(0.0);

            finalSpeed = speedometer.getSpeed();

            telemetry.addData("final speed is ", finalSpeed);

            if (Math.abs(finalSpeed) > speedThres) {
                powerList.add(power);
                speedList.add(finalSpeed);
            }
            if (powerList.size() > 1) {
                LinearFuncFitter ff = new LinearFuncFitter(speedList, powerList);
                LinearFunc fit = ff.fit();
                telemetry.addData("Ks", fit.alpha);
                telemetry.addData("Kv", fit.beta);
                telemetry.addData("R", fit.R2);
                telemetry.update();
            }

            telemetry.addLine("press b to restart with more power");
            telemetry.update();

            while (opModeIsActive()) {
                if (gamepad1.b)
                    break;
            }

            prevRead = axialDrive.getPosition(false);

            power += powerStep;
        }
    }
}