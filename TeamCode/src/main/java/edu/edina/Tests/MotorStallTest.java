package edu.edina.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import edu.edina.Libraries.Actions.ResetPosAction;

@Config
@TeleOp
@Disabled
public class MotorStallTest extends LinearOpMode {
    public static double voltage = 1;
    public static double ampLimit = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket t = new TelemetryPacket();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        Action a = new ResetPosAction(motor, hardwareMap.voltageSensor.iterator().next(), voltage, ampLimit);

        while (opModeIsActive()) {
            if (!a.run(new TelemetryPacket())) {
                a = null;
            } else {
                a.run(new TelemetryPacket());
            }


            t.put("position", motor.getCurrentPosition());
            t.put("current", motor.getCurrent(CurrentUnit.MILLIAMPS));
            dashboard.sendTelemetryPacket(t);
        }
    }
}
