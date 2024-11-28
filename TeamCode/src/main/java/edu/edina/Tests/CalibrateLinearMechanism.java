package edu.edina.Tests;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.LinearMotion.AxialDriveMechanism;
import edu.edina.Libraries.Robot.FuncInverter;
import edu.edina.Libraries.Robot.GamePadClick;
import edu.edina.Libraries.LinearMotion.ILinearMechanism;
import edu.edina.Libraries.Robot.LinearFunc;
import edu.edina.Libraries.Robot.LinearFuncFitter;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.Speedometer;

@TeleOp
//@Disabled
public class CalibrateLinearMechanism extends LinearOpMode {
    double powerStep = 0.02;
    double speedThres = 1;

    private GamePadClick click1;
    ILinearMechanism linearMech;

    @Override
    public void runOpMode() throws InterruptedException {
        linearMech = new AxialDriveMechanism(new RobotHardware(this));
        click1 = new GamePadClick(gamepad1);

        waitForStart();

        telemetry.addLine("Press a to calibrate encoder");
        telemetry.addLine("Press b to calibrate Kv and Ks");
        telemetry.addLine("Press x to calibrate Ka");
        telemetry.addLine("Press y to measure ambient acceleration");
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
        double power = powerStep;
        double ks = 0;
        double kv = 0;

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

            double maxSpeed = 0.0;
            while (opModeIsActive()) {
                linearMech.setPower(power);

                if (!gamepad1.dpad_up) {
                    break;
                }

                DualNum<Time> u = linearMech.getPositionAndVelocity(false);

                double s = u.get(1);
                if (s > maxSpeed)
                    maxSpeed = s;

                telemetry.addData("speed", s);
                telemetry.addData("power", power);
                if (ks != 0) {
                    double predictedSpeed = (power - ks) / kv;
                    telemetry.addData("predicted max speed", predictedSpeed);
                }

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
                ks = fit.alpha;
                kv = fit.beta;
                telemetry.addData("Ks", "%.4e", ks);
                telemetry.addData("Kv", "%.4e", kv);
                telemetry.addData("R", fit.R2);
            }

            telemetry.addLine("press down to back up");
            telemetry.addLine("press b to restart with more power");
            telemetry.update();

            while (opModeIsActive()) {
                if (gamepad1.dpad_down)
                    linearMech.setPower(-Math.min(power, 0.5));
                else
                    linearMech.setPower(0);

                if (gamepad1.b)
                    break;
            }

            power += powerStep;
        }
    }

    private void calibrateKa() {
        // assume that drive takes 1 second
        double targetDist = linearMech.getSettings().accelCalibrationDist;

        FuncInverter fi = new FuncInverter(targetDist, 0.1);

        fi.eval(0, testKaForSingleSecDist(0));
        fi.eval(0.3, testKaForSingleSecDist(0.3));

        while (opModeIsActive() && !fi.hasResult()) {
            double ka = fi.getGuess();
            double testDist = testKaForSingleSecDist(ka);
            fi.eval(ka, testDist);
        }

        telemetry.addData("final result", "ka = %.4f", fi.getResult());
        telemetry.update();
    }

    private double testKaForSingleSecDist(double ka) {
        // assume that ka is the correct value (it probably won't
        // be), and set the power based on that assumption. If you
        // are right, then the mechanism will actually go the expected
        // distance. The FuncInverter above will hopefully converge on
        // the correct value for ka so that this math becomes good.

        try {
            while (opModeIsActive()) {
                telemetry.addData("ka", "%.4e", ka);
                telemetry.addLine("press and hold up to test");
                telemetry.update();
                if (gamepad1.dpad_up)
                    break;
            }

            LinearMechanismSettings settings = linearMech.getSettings();

            double ks = settings.ks;
            double kv = settings.kv;
            double intendedAccel = 2 * settings.accelCalibrationDist;
            double startPos = linearMech.getPosition(false);
            ElapsedTime timer = new ElapsedTime();
            double t = 0;
            double dist = 0;

            while (opModeIsActive()) {
                if (!gamepad1.dpad_up)
                    throw new RuntimeException("test aborted");

                t = timer.seconds();
                if (t >= 1)
                    break;

                dist = linearMech.getPosition(false) - startPos;
                if (dist >= settings.accelCalibrationDist * 1.1)
                    break;

                double intendedVelocity = intendedAccel * t;
                double power = ks + kv * intendedVelocity + ka * intendedAccel;

                if (power > 1)
                    break;

                linearMech.setPower(power);
            }

            double actualA = 2.0 * (dist / (t * t));
            double singleSecondDistance = actualA / 2.0;

            telemetry.addData("target dist", settings.accelCalibrationDist);
            telemetry.addData("intended accel", intendedAccel);
            telemetry.addData("actual accel", actualA);
            telemetry.addData("ka", "%.4e", ka);
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

            return singleSecondDistance;
        } finally {
            linearMech.setPower(0);
        }
    }

    public double testAccel(double accel) {
        return 0.0;
    }
}