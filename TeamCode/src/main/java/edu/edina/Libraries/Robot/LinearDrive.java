package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import kotlin.NotImplementedError;

public class LinearDrive implements ILinearMechanism {
    private HardwareMap hw;
    private VoltageSensor vs;
    private DcMotorEx[] dcMotors;
    private double nominalVolt;

    //recalibrate
    private double ppi = 1564 / 38.125;
    private double inchesPerPos = 1.0 / ppi;

    public LinearDrive(HardwareMap hw) {
        this.hw = hw;

        dcMotors = new DcMotorEx[]{
                hw.get(DcMotorEx.class, "left_front_drive"),
                hw.get(DcMotorEx.class, "right_front_drive"),
                hw.get(DcMotorEx.class, "left_back_drive"),
                hw.get(DcMotorEx.class, "right_back_drive"),
        };

        for (int i = 0; i < 4; i++) {
            dcMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
        }

        vs = hw.voltageSensor.iterator().next();

        nominalVolt = 10.0;
    }

    public double getPpi() {
        return ppi;
    }

    @Override
    public void setPower(double power) {
        double adjPower = power * nominalVolt / vs.getVoltage();

        for (int i = 0; i < 4; i++) {
            dcMotors[i].setPower(adjPower);
        }
    }

    @Override
    public double getPosition(boolean raw) {
        if (raw) {
            return dcMotors[0].getCurrentPosition();
        } else {
            return dcMotors[0].getCurrentPosition() * inchesPerPos;
        }
    }

    @Override
    public double getKs() {
        throw new NotImplementedError();
    }

    @Override
    public double getKv() {
        throw new NotImplementedError();
    }
}