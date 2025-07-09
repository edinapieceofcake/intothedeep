package edu.edina.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.LinearMotion.VoltageCompensation;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;
import edu.edina.Libraries.NoisyFuncGuesser;
import edu.edina.Libraries.Quadratic;
import edu.edina.Libraries.Robot.Accelerometer;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.FuncInverter;
import edu.edina.Libraries.Robot.LinearFunc;
import edu.edina.Libraries.Robot.LinearFuncFitter;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.Robot.RobotState;

@TeleOp(name = "Calibrate Mechanism", group = "Test")
@Config
public class CalibrateLinearMechanism extends LinearOpMode {
    private static final String LOG_TAG = "linear cal";
    double powerStep = 0.02;
    double speedThres = 1;

    RobotState robotState;
    IMotionControlLinearMechanism linearMech;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket t = new TelemetryPacket();
        FtcDashboard f = FtcDashboard.getInstance();
        robotState = new RobotState(hardwareMap);
        linearMech = new Arm.Mechanism(robotState, hardwareMap);

        waitForStart();

        telemetry.addLine("Press a to calibrate encoder");
        telemetry.addLine("Press b to calibrate Kv and Ks");
        telemetry.addLine("Press x to calibrate Ka");
        telemetry.addLine("Press y to measure feed forward");
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
            if (gamepad1.y) {
                measureFeedForward();
            }
        }
    }

    private boolean isActive() {
        robotState.update(telemetry);
        return opModeIsActive();
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

    public static double MIN_FEED_FWD_POW = -0.2;
    public static double MAX_FEED_FWD_POW = 0.2;

    private void measureFeedForward() {
        double target = -1000;

        while (isActive()) {
            target = linearMech.getPositionAndVelocity(false).get(0);

            telemetry.addLine("move mechanism to wanted location, it will try and hold it there");
            telemetry.addData("target", "%.2f", target);
            telemetry.addLine("press b to start calibration");
            telemetry.update();

            if (gamepad1.b) break;
        }

        NoisyFuncGuesser fi = new NoisyFuncGuesser(target, MIN_FEED_FWD_POW, MAX_FEED_FWD_POW, 6);

        while (isActive() && !fi.hasResult()) {
            double guess = fi.getGuess();
            fi.eval(guess, testVelocityUnderCompensatedPower(guess));
        }

        while (opModeIsActive()) {
            telemetry.addData("target", "%.2f", target);
            telemetry.addData("final power", "%.2f", fi.getResult());
            telemetry.update();
        }
    }

    private double testVelocityUnderCompensatedPower(double power) {
        while (isActive()) {
            telemetry.addData("press and hold b to test with ", "%.4g power (vc)", power);
            telemetry.addData("position", linearMech.getPosition(false));
            telemetry.update();

            if (gamepad1.b) {
                break;
            }
        }

        VoltageCompensation vc = new VoltageCompensation(robotState);
        while (isActive()) {
            linearMech.setPower(vc.adjustPower(power));

            telemetry.addData("power", "%.2f", power);
            telemetry.addData("position", "%.2f", linearMech.getPosition(false));
            telemetry.update();

            if (!gamepad1.b) {
                break;
            }
        }

        linearMech.setPower(0);

        return linearMech.getPositionAndVelocity(false).get(1);
    }

    private static LinearFunc fitAccelFunc(List<Double> t, List<Double> x) {
        ArrayList<Double> xBatch = new ArrayList<>();
        Accelerometer accelerometer = new Accelerometer(3);

        ArrayList<Double> accelSample = new ArrayList<>();
        ArrayList<Double> accelSampleCenter = new ArrayList<>();

        // do batches of 10
        for (int i = 0; i < t.size(); i++) {
            accelerometer.sample(t.get(i), x.get(i));
            xBatch.add(x.get(i));

            if (xBatch.size() > 3) {
                accelSample.add(accelerometer.getAccel());
                accelSampleCenter.add(average(xBatch));

                xBatch.clear();
                accelerometer.clear();
            }
        }

        // skip the first
        accelSample.remove(0);
        accelSampleCenter.remove(0);

        for (int i = 1; i < accelSample.size(); i++) {
            RobotLog.ii(LOG_TAG, "accel sample: a = %.1f, x = %.1f", accelSample.get(i), accelSampleCenter.get(i));
        }

        // calculate gravity equation
        LinearFuncFitter f = new LinearFuncFitter(accelSampleCenter, accelSample);
        LinearFunc fit = f.fit();
        if (fit == null) throw new RuntimeException("Could not fit acceleration function");

        return fit;
    }

    private static double average(List<Double> xs) {
        double sum = 0;
        for (double x : xs)
            sum += x;
        return sum / xs.size();
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

                if (gamepad1.dpad_up) break;
            }

            double maxSpeed = 0.0;
            while (opModeIsActive()) {
                linearMech.setPower(power);

                if (!gamepad1.dpad_up) {
                    break;
                }

                DualNum<Time> u = linearMech.getPositionAndVelocity(false);

                double s = u.get(1);
                if (s > maxSpeed) maxSpeed = s;

                telemetry.addData("position", u.get(0));
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
                telemetry.addData("R^2", fit.R2);
            }

            telemetry.addLine("press down to back up");
            telemetry.addLine("press b to restart with more power");
            telemetry.update();

            while (opModeIsActive()) {
                if (gamepad1.dpad_down) linearMech.setPower(-Math.min(power, 0.5));
                else linearMech.setPower(0);

                if (gamepad1.b) break;
            }

            power += powerStep;
        }
    }

    private void calibrateKa() {
        // assume that drive takes 1 second
        LinearMechanismSettings settings = linearMech.getSettings();
        FuncInverter fi = new FuncInverter(1, 0.05);

        double ka0 = 0.3 * settings.ka;
        double ka1 = 3 * settings.ka;

        fi.eval(ka0, testKaAccelRatio(ka0));
        fi.eval(ka1, testKaAccelRatio(ka1));

        while (opModeIsActive() && !fi.hasResult()) {
            double ka = fi.getGuess();
            double testDist = testKaAccelRatio(ka);
            fi.eval(ka, testDist);
        }

        while (opModeIsActive()) {
            telemetry.addData("final result", "ka = %.4e", fi.getResult());
            telemetry.update();
        }
    }

    private double testKaAccelRatio(double ka) {
        // Try to drive a distance of accelCalibrationDist. Using the guess for ka,
        // determine how long it should drive, apply the acceleration

        LinearMechanismSettings settings = linearMech.getSettings();
        double d = linearMech.getAccelCalDistance();
        double nominalTime = Quadratic.rootOrDefault(0.5, -2 * settings.kv * d, -settings.ks - 2 * d * ka, 0);

        if (nominalTime == 0) throw new RuntimeException("zero time test");

        double intendedAccel = 2 * d / (nominalTime * nominalTime);

        try {
            while (opModeIsActive()) {
                telemetry.addData("ka", "%.4e", ka);
                telemetry.addData("nominal test time", "%.2f", nominalTime);
                telemetry.addLine("press and hold up to test");
                telemetry.update();
                if (gamepad1.dpad_up) break;
            }

            double ks = settings.ks;
            double kv = settings.kv;
            double startPos = linearMech.getPositionAndVelocity(false).get(0);
            ElapsedTime timer = new ElapsedTime();
            double t = 0;
            double dist = 0;

            while (opModeIsActive()) {
                if (!gamepad1.dpad_up)
                    throw new RuntimeException("test aborted");

                t = timer.seconds();

                DualNum<Time> u = linearMech.getPositionAndVelocity(false);
                dist = u.get(0) - startPos;
                if (dist >= settings.accelCalibrationDist) {
                    break;
                }

                double v = u.get(1);
                double power = ks + kv * v + ka * intendedAccel;

                if (power > 1) {
                    break;
                }

                linearMech.setPower(power);
            }

            double actualA = 2.0 * (dist / (t * t));

            telemetry.addData("ka", "%.4e", ka);
            telemetry.addData("test complete", "time=%.2f (nominal=%.2f)", t, nominalTime);
            telemetry.addData("dist", "target %.2f, actual %.2f", settings.accelCalibrationDist, dist);
            telemetry.addData("intended accel", intendedAccel);
            telemetry.addData("actual accel", actualA);
            telemetry.addData("ratio (goal is 1)", "%.4f", actualA / intendedAccel);
            telemetry.addLine("press down to reset");
            telemetry.addLine("press x to continue calibrating");
            telemetry.update();

            while (opModeIsActive()) {
                if (gamepad1.dpad_down) linearMech.setPower(-0.2);
                else linearMech.setPower(0);

                if (gamepad1.x) break;
            }

            return actualA / intendedAccel;
        } finally {
            linearMech.setPower(0);
        }
    }
}