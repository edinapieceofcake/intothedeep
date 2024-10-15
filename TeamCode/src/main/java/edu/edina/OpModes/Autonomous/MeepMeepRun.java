package edu.edina.OpModes.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.RoadRunner.MecanumDrive;

@Autonomous
public class MeepMeepRun extends LinearOpMode  {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean sampleSide = true;

        if (sampleSide) {
            // *****SAMPLES*****

            // PASTE THIS LINE
            Pose2d beginPose = new Pose2d(-50, -60, 0);
            // STOP PASTING

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)

                            // NEUTRAL SAMPLES
                            // PASTE HERE
                            .waitSeconds(1)
                            .strafeToLinearHeading(new Vector2d(-48, -38), 1.0 / 2 * Math.PI)
                            .waitSeconds(0.5)
                            .strafeToLinearHeading(new Vector2d(-60, -50), 1.0 / 2 * Math.PI)
                            .waitSeconds(1)
                            .strafeToLinearHeading(new Vector2d(-58, -38), 1.0 / 2 * Math.PI)
                            .waitSeconds(0.5)
                            .strafeToLinearHeading(new Vector2d(-60, -50), 1.0 / 2 * Math.PI)
                            .waitSeconds(1)
                            .strafeToLinearHeading(new Vector2d(-57, -25), Math.PI)
                            .waitSeconds(0.5)
                            .strafeToLinearHeading(new Vector2d(-60, -50), 1.0 / 2 * Math.PI)
                            .waitSeconds(1)
                            .setReversed(false)
                            .splineTo(new Vector2d(-22, 0), 0)
                            .waitSeconds(1)
                            .setReversed(true)
                            .splineTo(new Vector2d(-60, -50), 3.0 / 2 * Math.PI)
                            .waitSeconds(1)
                            .setReversed(false)
                            .splineTo(new Vector2d(-22, 0), 0)
                            .waitSeconds(1)
                            .setReversed(true)
                            .splineTo(new Vector2d(-60, -50), 3.0 / 2 * Math.PI)
                            .waitSeconds(1)
                            .setReversed(false)
                            .splineTo(new Vector2d(-22, 0), 0)
                            .waitSeconds(1)
                            .setReversed(true)
                            .splineTo(new Vector2d(-60, -50), 3.0 / 2 * Math.PI)
                            // STOP PASTING

                            .build());

        }
        else {
            // *****SPECIMENS*****

            // PASTE THIS LINE
            Pose2d beginPose = new Pose2d(0, -60, 3.0/2*Math.PI);
            // STOP PASTING

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)

                            // NEUTRAL SAMPLES
                            // PASTE HERE
                            .strafeTo(new Vector2d(0,-35))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(47,-60))
                            .waitSeconds(0.5)
                            .strafeTo(new Vector2d(0,-35))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(47,-60))
                            .waitSeconds(0.5)
                            .strafeTo(new Vector2d(0,-35))
                            .waitSeconds(1)
                            .setReversed(false)
                            .splineTo(new Vector2d(38,-25), 0)
                            .waitSeconds(0.5)
                            .strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI)
                            .waitSeconds(0.5)
                            .strafeTo(new Vector2d(0,-35))
                            .waitSeconds(1)
                            .setReversed(false)
                            .splineTo(new Vector2d(48,-25), 0)
                            .waitSeconds(0.5)
                            .strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI)
                            .waitSeconds(0.5)
                            .strafeTo(new Vector2d(0,-35))
                            .waitSeconds(1)
                            .setReversed(false)
                            .splineTo(new Vector2d(58,-25), 0)
                            .waitSeconds(0.5)
                            .strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI)
                            .waitSeconds(0.5)
                            .strafeTo(new Vector2d(0,-35))
                            .waitSeconds(1)
                            .strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI)
                            .waitSeconds(0.5)
                            .strafeTo(new Vector2d(0,-35))
                            .waitSeconds(1)
                            // STOP PASTING

                            .build());

        }
    }
}
