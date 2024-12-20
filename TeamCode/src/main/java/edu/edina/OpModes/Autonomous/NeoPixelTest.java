package edu.edina.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Random;

import edu.edina.Libraries.Robot.NeoPixelDriverDevice;

//@Disabled
@TeleOp
public class NeoPixelTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeoPixelDriverDevice neoPixel = hardwareMap.get(NeoPixelDriverDevice.class, "neopixel_driver");

        waitForStart();

        byte[] pixArray = new byte[NeoPixelDriverDevice.NUM_BYTES];

        Random r = new Random();

        boolean enable_red = true;
        boolean enable_green = true;
        boolean enable_blue = true;

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                enable_red = false;
                enable_green = true;
                enable_blue = false;
                telemetry.addData("color", "green");
                telemetry.update();
            }

            if (gamepad1.b) {
                enable_red = true;
                enable_green = false;
                enable_blue = false;
                telemetry.addData("color", "red");
                telemetry.update();
            }

            if (gamepad1.x) {
                enable_red = false;
                enable_green = false;
                enable_blue = true;
                telemetry.addData("color", "blue");
                telemetry.update();
            }

            if (gamepad1.y) {
                enable_red = true;
                enable_green = true;
                enable_blue = true;
                telemetry.addData("color", "rgb");
                telemetry.update();
            }

            for (int i = 0; i < pixArray.length; i += 3) {
                pixArray[i] = enable_green ? (byte) r.nextInt() : 0;
            }

            for (int i = 1; i < pixArray.length; i += 3) {
                pixArray[i] = enable_red ? (byte) r.nextInt() : 0;
            }

            for (int i = 2; i < pixArray.length; i += 3) {
                pixArray[i] = enable_blue ? (byte) r.nextInt() : 0;
            }

            neoPixel.showColors(pixArray);

            telemetry.addData("time", t);
            telemetry.addData("numWrites", neoPixel.getNumWrites());
            telemetry.update();
        }
    }
}