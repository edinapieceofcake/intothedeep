package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.WaitForTime;

@Config
public class Ascent {
    private static final String tag = "Ascent";
    private boolean abort;

    public static int SERVO_DROP_POS = 1150;
    public static int LIFT_REVERSE_POS = 1250;
    public static int SERVO_HOOK_POS = 1050;
    public static double LIFT_FEEDFORWARD_1 = 0.0002;
    public static double LIFT_FEEDFORWARD_2 = 0.06;
    public static double DOWNWARD_POWER_ADDON = -0.45;
    public static double RAISE_POWER_ADDON = 0.7;
    public static double DOWNWARD_POWER_RAW = -0.2; // No feedforward
    public static double HOOK_AMP_LIMIT = 1.1;
    public static double HANG_POWER_RAMP = 2;

    private double initialVolt;
    private ServoImplEx hangLeft, hangRight;
    private DcMotorEx leftLift, rightLift;
    private RobotHardware hw;
    private int initPos;
    private final ElapsedTime hangTime;

    public Ascent(RobotHardware hw) {
        this.hw = hw;
        hangLeft = hw.getOpMode().hardwareMap.get(ServoImplEx.class, "hang_left");
        hangRight = hw.getOpMode().hardwareMap.get(ServoImplEx.class, "hang_right");
        leftLift = hw.getOpMode().hardwareMap.get(DcMotorEx.class, "left_lift_motor");
        rightLift = hw.getOpMode().hardwareMap.get(DcMotorEx.class, "right_lift_motor");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        initialVolt = hw.getVoltageSensor().getVoltage();
        initPos = getLiftPos();
        abort = false;
        hangTime = new ElapsedTime();
    }

    public Action turnOffServos() {
        return new InstantAction(() -> {
            hw.getDualClaw().turnOff();
            hw.getSwivel().turnOff();
            hw.getWrist().turnOff();
        });
    }

    public Action lowerHook() {
        return new InstantAction(() -> {
            hangLeft.setPosition(.3);
            hangRight.setPosition(.7);
        });
    }

    public Action raiseLift() {
        int minPos = initPos + SERVO_DROP_POS;
        return liftMotionAction(minPos, RAISE_POWER_ADDON);
    }

    public Action lowerLift() {
        int hookOnThresh = initPos + SERVO_HOOK_POS;
        return liftMotionAction(hookOnThresh, DOWNWARD_POWER_ADDON);
    }

    private Action liftMotionAction(int cutoff, double powerConst) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                int p = getLiftPos();

                RobotLog.ii(tag, "lift pos %d", p);

                double feedfwd = LIFT_FEEDFORWARD_1 * p + LIFT_FEEDFORWARD_2;

                if ((powerConst > 0 && p < cutoff) || (powerConst < 0 && p > cutoff)) {
                    RobotLog.ii(tag, "lift pos %d, power %f", p, powerConst + feedfwd);
                    setPower(powerConst + feedfwd);
                    return true;
                } else {
                    setPower(feedfwd);
                    return false;
                }
            }
        };
    }

    public Action moveArmToAllowLiftsToRaise() {
        Arm a = hw.getArm();
        return new InstantAction(() -> a.setPosition(a.getCurrentPosition() + 800));
    }

    public Action moveArmToAllowLiftsToLower() {
        Arm a = hw.getArm();
        return new InstantAction(() -> a.setPosition(a.getCurrentPosition() - 750));
    }

    public Action waitForHook() {
        Action initAction = new InstantAction(() -> {
            setPower(DOWNWARD_POWER_RAW);
        });
        Action wait = new WaitForTime(20);
        Action hookDetect = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double current = getCurrent();
                int pos = getLiftPos();

                RobotLog.ii(tag, "pos = %d, current = %f", pos, current);

                if (current > HOOK_AMP_LIMIT) {
                    return false;
                } else {
                    if (pos < initPos + SERVO_DROP_POS / 2) {
                        abort = true;
                        return false;
                    } else {
                        return true;
                    }
                }
            }
        };

        return new SequentialAction(
                initAction,
                wait,
                hookDetect
        );
    }

    public Action hang() {
        Action startHang = new InstantAction(hangTime::reset);
        Action hang = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (abort) {
                    RobotLog.ii(tag, "hang aborted");
                    setPower(0);
                    return false;
                } else {
                    RobotLog.ii(tag, "hanging");
                    hw.getSlide().setPosition(1000);
                    setPower(-HANG_POWER_RAMP * hangTime.seconds());
                    return true;
                }
            }
        };

        return new SequentialAction(startHang, hang);
    }

    private void setPower(double power) {
        double actualPower = power * 12 / initialVolt;
        leftLift.setPower(actualPower);
        rightLift.setPower(actualPower);
    }

    private int getLiftPos() {
        return (leftLift.getCurrentPosition() - rightLift.getCurrentPosition()) / 2;
    }

    public Action turnOffHook() {
        return new InstantAction(() -> {
            hangRight.setPwmDisable();
            hangLeft.setPwmDisable();
        });
    }

    private double getCurrent() {
        return (leftLift.getCurrent(CurrentUnit.AMPS) + rightLift.getCurrent(CurrentUnit.AMPS)) / 2;
    }

    public Action engageHook() {
        return new InstantAction(() -> {
            hangLeft.setPwmEnable();
            hangRight.setPwmEnable();
            hangRight.setPosition(1);
            hangLeft.setPosition(0);
        });
    }
}