package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.Drivetrain;

public class FieldCentricTest extends LinearOpMode {
    private Drivetrain dt;
    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this);

        waitForStart();

        while (opModeIsActive()) {

        }
    }


}