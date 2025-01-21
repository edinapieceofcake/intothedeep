package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ResetPosAction implements Action {
    //Current = Voltage / Resistance
    //Example Thresh values
    //1.6V = 2Amps
    //3.8V = 4Amps

    //amp limit zeros

    private DcMotorEx motor;
    private VoltageSensor vs;
    private double voltage, ampLimit;
    private DcMotor.RunMode mode;

    public ResetPosAction(DcMotorEx motor, VoltageSensor vs, double voltage, double ampLimit) {
        this.motor = motor;
        this.vs = vs;
        this.ampLimit = ampLimit;
        this.voltage = voltage;
        mode = motor.getMode();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double power = voltage / vs.getVoltage();
        motor.setPower(power);
        double amps = motor.getCurrent(CurrentUnit.AMPS);

        if ((ampLimit > 0 && amps > ampLimit) || (ampLimit < 0 && amps < ampLimit)) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(mode);
            return false;
        } else {
            return true;
        }
    }
}
