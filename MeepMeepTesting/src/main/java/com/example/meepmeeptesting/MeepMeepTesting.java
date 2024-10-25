package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Only for meepmeep
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        boolean sampleSide = true;

        if (sampleSide) {
            // *****SAMPLES*****

            // COPY THIS LINE
            Pose2d beginPose = new Pose2d(-38, -62, 0);
            // COPY THIS LINE

            myBot.runAction(myBot.getDrive().actionBuilder(beginPose)

                    // COPY HERE
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
                    // STOP COPYING

                    .build());


        }
        else {
            // *****SPECIMENS*****

            // START COPYING
            Pose2d beginPose = new Pose2d(0, -61.5, 3.0/2*Math.PI);
            // STOP COPYING

            myBot.runAction(myBot.getDrive().actionBuilder(beginPose)

                    // COPY HERE
                    .strafeTo(new Vector2d(0,-35))
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(47,-50))
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(0,-35))
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(47,-50))
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(0,-35))
                    .waitSeconds(1)
                    .setReversed(false)
                    .splineTo(new Vector2d(38,-25), 0)
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(47,-50), 3.0/2*Math.PI)
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(0,-35))
                    .waitSeconds(1)
                    .setReversed(false)
                    .splineTo(new Vector2d(48,-25), 0)
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(47,-50), 3.0/2*Math.PI)
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(0,-35))
                    .waitSeconds(1)
                    .setReversed(false)
                    .splineTo(new Vector2d(58,-25), 0)
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(47,-50), 3.0/2*Math.PI)
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(0,-35))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(47,-50), 3.0/2*Math.PI)
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(0,-35))
                    .waitSeconds(1)
                    // STOP COPYING


                    // USE SPLINES INSTEAD IF USEFUL
                    /*
                    .setReversed(false)
                    .splineTo(new Vector2d(50,-60), (double)3/2*Math.PI)
                    .setReversed(true)
                    .splineTo(new Vector2d(0,-35), (double)1/2*Math.PI)
                    .setReversed(false)
                    .splineTo(new Vector2d(50,-60), (double)3/2*Math.PI)
                    .setReversed(true)
                    .splineTo(new Vector2d(0,-35), (double)1/2*Math.PI)
                   */

                    .build());
        }

        // Only for meepmeep
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .addEntity(myBot)
                .start();

    }
}