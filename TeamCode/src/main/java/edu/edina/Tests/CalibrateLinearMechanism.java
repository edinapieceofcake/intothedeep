package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.Robot.ArmExtension;
import edu.edina.Libraries.Robot.ArmLift;
import edu.edina.Libraries.Robot.FuncInverter;
import edu.edina.Libraries.Robot.GamePadClick;
import edu.edina.Libraries.Robot.ILinearMechanism;
import edu.edina.Libraries.Robot.LinearFunc;
import edu.edina.Libraries.Robot.LinearFuncFitter;
import edu.edina.Libraries.Robot.LinearMechanismSettings;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.Speedometer;

@TeleOp
public class CalibrateLinearMechanism extends LinearOpMode {
    double powerStep = 0.02;
    double speedThres = 1;

    private GamePadClick click1;
    ILinearMechanism linearMech;

    @Override
    public void runOpMode() {
        linearMech = new ArmExtension(new RobotHardware(hardwareMap));
        click1 = new GamePadClick(gamepad1);

        waitForStart();

        telemetry.addLine("Press a to calibrate encoder");
        telemetry.addLine("Press b to calibrate Kv and Ks");
        telemetry.addLine("Press x to calibrate Ka");
        telemetry.addLine("Press y to calibrate acceleration");
        telemetry.update();

        while (opModeIsActive()) {
            click1.read();

            if (click1.a) {
                calibrateEncoder();
            }
            if (click1.b) {
                calibrateLinearKs();
            }
            if (click1.x) {
                calibrateKa();
            }
            if (click1.y) {
            }
        }
    }

    private void calibrateEncoder() {
        while (opModeIsActive()) {
            telemetry.addLine("press up to move forward");

            if (gamepad1.dpad_up) {
                linearMech.setPower(0.3);
            } else if (gamepad1.dpad_down) {
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
        double maxSpeed = 0.0;

        List<Double> powerList = new ArrayList<>();
        List<Double> speedList = new ArrayList<>();

        while (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addLine("press and hold up to start test");
                telemetry.addData("power", power);
                telemetry.update();

                if (gamepad1.dpad_up)
                    break;
            }

            while (opModeIsActive()) {
                linearMech.setPower(power);

                if (!gamepad1.dpad_up) {
                    break;
                }

                speedometer.sample(linearMech.getPosition(false) - prevRead);

                double s = speedometer.getSpeed();
                if (s > maxSpeed)
                    maxSpeed = s;

                telemetry.addData("speed", s);
                telemetry.addData("power", power);
                telemetry.update();
            }

            linearMech.setPower(0.0);

            telemetry.addData("max speed is ", maxSpeed);

            if (Math.abs(maxSpeed) > speedThres) {
                powerList.add(power);
                speedList.add(maxSpeed);
            }

            LinearFuncFitter ff = new LinearFuncFitter(speedList, powerList);
            LinearFunc fit = ff.fit();

            if (fit != null) {
                telemetry.addData("Ks", fit.alpha);
                telemetry.addData("Kv", fit.beta);
                telemetry.addData("R", fit.R2);
            }

            telemetry.addLine("press down to back up");
            telemetry.addLine("press b to restart with more power");
            telemetry.update();

            while (opModeIsActive()) {
                if (gamepad1.dpad_down)
                    linearMech.setPower(-power);
                else
                    linearMech.setPower(0);

                if (gamepad1.b)
                    break;
            }

            prevRead = linearMech.getPosition(false);

            power += powerStep;
        }
    }

    private void calibrateKa() {
        // assume that drive takes 1 second
        double targetDist = linearMech.getSettings().accelCalibrationTarget / 2;

        FuncInverter fi = new FuncInverter(targetDist, 0.1);

        fi.eval(0, testKa(0, targetDist));
        fi.eval(0.5, testKa(0.5, targetDist));

        while (!fi.hasResult()) {
            double ka = fi.getGuess();
            double testDist = testKa(ka, targetDist);
            fi.eval(ka, testDist);
        }

        telemetry.addData("final result", "ka = %.4f", fi.getResult());
        telemetry.update();
    }

    private double testKa(double ka, double targetDist) {
        // assume that ka is the correct value (it probably won't
        // be), and set the power based on that assumption. If you
        // are right, then the mechanism will actually go the expected
        // distance. The FuncInverter above will hopefully converge on
        // the correct value for ka so that this math becomes good.

        while (opModeIsActive()) {
            telemetry.addLine("press and hold up to test");
            telemetry.update();
            if (gamepad1.dpad_up)
                break;
        }

        LinearMechanismSettings settings = linearMech.getSettings();

        double ks = linearMech.getSettings().ks;
        double kv = linearMech.getSettings().kv;
        double a = linearMech.getSettings().accelCalibrationTarget;
        double startPos = linearMech.getPosition(false);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            if (!gamepad1.dpad_up) break;

            double t = timer.seconds();
            double derivedVelocityGuess = a * t;
            double power = ks + kv * derivedVelocityGuess + ka * a;
            linearMech.setPower(power);
        }

        double endPos = linearMech.getPosition(false);

        linearMech.setPower(0);

        double dist = endPos - startPos;

        telemetry.addData("target dist", targetDist);
        telemetry.addData("actual dist", dist);
        telemetry.addLine("press down to reset");
        telemetry.addLine("press x to continue calibrating");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down)
                linearMech.setPower(-0.2);
            else
                linearMech.setPower(0);

            if (gamepad1.x)
                break;
        }

        return dist;
    }

    public double testAccel(double accel) {
        return 0.0;
    }
}