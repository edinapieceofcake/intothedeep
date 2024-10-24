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

            sleep(1000);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(-48, -38), 1.0 / 2 * Math.PI).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(-60, -50), 1.0 / 2 * Math.PI).build());
            sleep(1000);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(-58, -38), 1.0 / 2 * Math.PI).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(-60, -50), 1.0 / 2 * Math.PI).build());
            sleep(1000);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(-57, -25), Math.PI).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(-60, -50), 1.0 / 2 * Math.PI).build());
            for (int i = 0; i < 3; i++)
            {
                sleep(1000);
                Actions.runBlocking(drive.actionBuilder(beginPose).setReversed(false).splineTo(new Vector2d(-22, 0), 0).build());
                sleep(1000);
                Actions.runBlocking(drive.actionBuilder(beginPose).setReversed(true).splineTo(new Vector2d(-60, -50), 3.0 / 2 * Math.PI).build());
            }
            // WITHOUT SLEEPS
            /*
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)

                            // NEUTRAL SAMPLES
                            // PASTE HERE
                            .strafeToLinearHeading(new Vector2d(-48, -38), 1.0 / 2 * Math.PI)
                            .strafeToLinearHeading(new Vector2d(-60, -50), 1.0 / 2 * Math.PI)
                            .strafeToLinearHeading(new Vector2d(-58, -38), 1.0 / 2 * Math.PI)
                            .strafeToLinearHeading(new Vector2d(-60, -50), 1.0 / 2 * Math.PI)
                            .strafeToLinearHeading(new Vector2d(-57, -25), Math.PI)
                            .strafeToLinearHeading(new Vector2d(-60, -50), 1.0 / 2 * Math.PI)
                            .setReversed(false)
                            .splineTo(new Vector2d(-22, 0), 0)
                            .setReversed(true)
                            .splineTo(new Vector2d(-60, -50), 3.0 / 2 * Math.PI)
                            .setReversed(false)
                            .splineTo(new Vector2d(-22, 0), 0)
                            .setReversed(true)
                            .splineTo(new Vector2d(-60, -50), 3.0 / 2 * Math.PI)
                            .setReversed(false)
                            .splineTo(new Vector2d(-22, 0), 0)
                            .setReversed(true)
                            .splineTo(new Vector2d(-60, -50), 3.0 / 2 * Math.PI)
                            // STOP PASTING

                            .build());
             */
        }
        else {
            // *****SPECIMENS*****

            // PASTE THIS LINE
            Pose2d beginPose = new Pose2d(0, -60, 3.0/2*Math.PI);
            // STOP PASTING

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(drive.actionBuilder(beginPose).strafeTo(new Vector2d(0,-35)).build());
            sleep(1000);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeTo(new Vector2d(47,-60)).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeTo(new Vector2d(0,-35)).build());
            sleep(1000);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeTo(new Vector2d(47,-60)).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeTo(new Vector2d(0,-35)).build());
            sleep(1000);
            Actions.runBlocking(drive.actionBuilder(beginPose).setReversed(false).splineTo(new Vector2d(38,-25), 0).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeTo(new Vector2d(0,-35)).build());
            sleep(1000);
            Actions.runBlocking(drive.actionBuilder(beginPose).setReversed(false).splineTo(new Vector2d(48,-25), 0).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeTo(new Vector2d(0,-35)).build());
            sleep(1000);
            Actions.runBlocking(drive.actionBuilder(beginPose).setReversed(false).splineTo(new Vector2d(58,-25), 0).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeTo(new Vector2d(0,-35)).build());
            sleep(1000);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI).build());
            sleep(500);
            Actions.runBlocking(drive.actionBuilder(beginPose).strafeTo(new Vector2d(0,-35)).build());
            sleep(1000);

            // WITHOUT DELAYS
            /*
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)

                            // NEUTRAL SAMPLES
                            // PASTE HERE
                            .strafeTo(new Vector2d(0,-35))
                            .strafeTo(new Vector2d(47,-60))
                            .strafeTo(new Vector2d(0,-35))
                            .strafeTo(new Vector2d(47,-60))
                            .strafeTo(new Vector2d(0,-35))
                            .setReversed(false)
                            .splineTo(new Vector2d(38,-25), 0)
                            .strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI)
                            .strafeTo(new Vector2d(0,-35))
                            .setReversed(false)
                            .splineTo(new Vector2d(48,-25), 0)
                            .strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI)
                            .strafeTo(new Vector2d(0,-35))
                            .setReversed(false)
                            .splineTo(new Vector2d(58,-25), 0)
                            .strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI)
                            .strafeTo(new Vector2d(0,-35))
                            .strafeToLinearHeading(new Vector2d(47,-60), 3.0/2*Math.PI)
                            .strafeTo(new Vector2d(0,-35))
                            // STOP PASTING

                            .build());
             */
        }
    }
}