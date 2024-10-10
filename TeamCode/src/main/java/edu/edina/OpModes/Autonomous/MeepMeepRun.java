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
        // PASTE THIS LINE
        Pose2d beginPose = new Pose2d(-50, -60, 0);
        // STOP PASTING

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        // PASTE HERE
                        .strafeToLinearHeading(new Vector2d(-48,-38), (double)1/2*Math.PI)
                        .strafeToLinearHeading(new Vector2d(-60,-50), (double)1/2*Math.PI)
                        .strafeToLinearHeading(new Vector2d(-58,-38), (double)1/2*Math.PI)
                        .strafeToLinearHeading(new Vector2d(-60,-50), (double)1/2*Math.PI)
                        .strafeToLinearHeading(new Vector2d(-57,-25), Math.PI)
                        .strafeToLinearHeading(new Vector2d(-60,-50), (double)1/2*Math.PI)
                        // STOP PASTING

                        .build());
    }
}
