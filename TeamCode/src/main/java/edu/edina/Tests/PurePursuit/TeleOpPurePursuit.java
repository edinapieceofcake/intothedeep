package edu.edina.Tests.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import roadrunner.ThreeDeadWheelLocalizer;

@TeleOp
public class TeleOpPurePursuit extends LinearOpMode {
    private IMU imu;
    private ThreeDeadWheelLocalizer odometry;


    @Override
    public void runOpMode() {


        while (opModeIsActive()) {

        }
    }
}
