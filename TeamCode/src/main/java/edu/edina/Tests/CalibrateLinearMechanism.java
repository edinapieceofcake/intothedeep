package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.Robot.ArmExtension;
import edu.edina.Libraries.Robot.ILinearMechanism;
import edu.edina.Libraries.Robot.LinearFunc;
import edu.edina.Libraries.Robot.LinearFuncFitter;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.Speedometer;

@TeleOp
public class CalibrateLinearMechanism extends LinearOpMode {
    double powerStep = 0.02;
    double speedThres = 1;

    ILinearMechanism linearMech;

    @Override
    public void runOpMode() {
        linearMech = new ArmExtension(new RobotHardware(hardwareMap));

        waitForStart();

        telemetry.addLine("Press a to calibrate encoder");
        telemetry.addLine("Press b to calibrate Kv and Ks");
        telemetry.addLine("Press x to calibrate Ka");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                calibrateEncoder();
            }
            if (gamepad1.b) {
                calibrateLinearKs();
            }
            if (gamepad1.x) {
                calibrateKa();
            }
        }
    }

    private void calibrateEncoder() {
        while (opModeIsActive()) {
            telemetry.addLine("press up to drive forward");

            if (gamepad1.dpad_up) {
                linearMech.setPower(0.3);
            }else if(gamepad1.dpad_down){
                linearMech.setPower(-0.3);
            } else {
                linearMech.setPower(0.0);
            }

            telemetry.addData("encoder (raw)", linearMech.getPosition(true));
            telemetry.addData("encoder (inch)", linearMech.getPosition(false));
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
                linearMech.setPower(power);

                if (gamepad1.start) {
                    break;
                }

                speedometer.sample(linearMech.getPosition(false) - prevRead);

                telemetry.addData("speed", speedometer.getSpeed());
                telemetry.addData("power", power);
                telemetry.update();
            }

            linearMech.setPower(0.0);

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

            prevRead = linearMech.getPosition(false);

            power += powerStep;
        }
    }

    private void calibrateKa() {
//        double min

    }

    private  void testKa(double ka){

    }
}