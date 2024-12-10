package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Only for meepmeep
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        boolean sampleSide = false;

        if (sampleSide) {
            // *****SAMPLES*****

            // COPY THIS LINE
            Pose2d beginPose = new Pose2d(-38, -62, 0);
            // COPY THIS LINE

            myBot.runAction(myBot.getDrive().actionBuilder(beginPose)
                    // COPY HERE
                    .strafeToLinearHeading(new Vector2d(-50, -50), 1.0 / 4 * Math.PI)
                    .strafeToLinearHeading(new Vector2d(-47.5, -38), 1.0 / 2 * Math.PI)
                    .strafeToLinearHeading(new Vector2d(-50, -50), 1.0 / 4 * Math.PI)
                    .strafeToLinearHeading(new Vector2d(-57.5, -44), 1.0 / 2 * Math.PI)
                    .strafeToLinearHeading(new Vector2d(-57.5, -38), 1.0 / 2 * Math.PI)
                    .strafeToLinearHeading(new Vector2d(-50, -50), 1.0 / 4 * Math.PI)
                    .strafeToLinearHeading(new Vector2d(-57, -25), Math.PI)


                    //.splineToLinearHeading(new Pose2d(-57.5, -38, 1.0 / 2 * Math.PI), 1.0 / 2 * Math.PI)

                    /*
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
                    */
                    // STOP COPYING
                    .build());


        }
        else {
            // *****SPECIMENS*****

            // START COPYING
            Pose2d beginPose = new Pose2d(0, -61.5, Math.toRadians(180));
            // STOP COPYING

            myBot.runAction(myBot.getDrive().actionBuilder(beginPose)

                    // COPY HERE
                    // High Rung Position
                    .strafeToLinearHeading(new Vector2d(0,-35), Math.toRadians(270))
                    //.strafeToLinearHeading(new Vector2d(35,-35), Math.toRadians(270))
                    //.strafeToLinearHeading(new Vector2d(35,-35), Math.toRadians(270))
                    .setTangent(Math.toRadians(0))

                    //.splineToLinearHeading(new Pose2d(37,-24,Math.toRadians(270)), Math.toRadians(90))
                    //.splineTo(new Vector2d(37,-24), Math.toRadians(90))//, Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36,-24), Math.toRadians(90))//, Math.toRadians(90))

                    //.setTangent(Math.toRadians(90))

                    //.splineToLinearHeading(new Pose2d(42,-12,Math.toRadians(270)), Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(42,-16), Math.toRadians(0)) //, Math.toRadians(270))

                    //.strafeToLinearHeading(new Vector2d(36,-35), Math.toRadians(270))
                    //.strafeToLinearHeading(new Vector2d(36,-15), Math.toRadians(270))
                    //.setTangent(Math.toRadians(90))
                    //.splineToLinearHeading(new Pose2d(46,-10, Math.toRadians(270)), Math.toRadians(0))
                    //.setTangent(Math.toRadians(270))

                    //.splineToLinearHeading(new Pose2d(48,-55,Math.toRadians(270)), Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(48,-43), Math.toRadians(270))

                    //.strafeTo(new Vector2d(46,-55))
                    /*.strafeTo(new Vector2d(48,-10))
                    .strafeTo(new Vector2d(57,-10))
                    .strafeTo(new Vector2d(57,-55))
                    .strafeTo(new Vector2d(57,-10))
                    .strafeTo(new Vector2d(61,-10))
                    .strafeTo(new Vector2d(61,-55))
//                                  end here
                                    .strafeTo(new Vector2d(61,-50))
                            .strafeTo(new Vector2d(55,-50))
                                    .strafeTo(new Vector2d(55,-55))
                    .strafeToLinearHeading(new Vector2d(0,-35), Math.toRadians(270))
                    .strafeTo(new Vector2d(55,-55))
                    .strafeToLinearHeading(new Vector2d(2,-35), Math.toRadians(270))
                    .strafeTo(new Vector2d(55,-55))
                    .strafeToLinearHeading(new Vector2d(4,-35), Math.toRadians(270))
                    .strafeTo(new Vector2d(55,-55))
                    .strafeToLinearHeading(new Vector2d(6,-35), Math.toRadians(270))
                    .strafeTo(new Vector2d(55,-55))
                    .strafeToLinearHeading(new Vector2d(8,-35), Math.toRadians(270))*/



                    //.splineToLinearHeading(new Pose2d(47,-10, Math.toRadians(90)), Math.toRadians(0))
                    /*.setTangent(15.0/8*Math.PI)
                    .splineToLinearHeading(new Pose2d(47,-37,Math.toRadians(89)), Math.toRadians(89))
                    .strafeToLinearHeading(new Vector2d(52, -45), 7.0/4*Math.PI)
                    .turnTo(3.0/2*Math.PI)
                    .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(270))*/

//                    .splineTo(new Vector2d(20, -35), 1.0/4*Math.PI)
//                    .splineTo(new Vector2d(35, -25), 0)

//                    //First Spike
//                            .splineTo(new Vector2d(48.4,-36),Math.toRadians(90))
//                    .waitSeconds(0.5)
//                    .setReversed(false)
//                    //Human Player
//                    .strafeTo(new Vector2d(47,-50))
//                    .strafeToLinearHeading(new Vector2d(47,-46.5), 3.0/2*Math.PI)
//                    .strafeToLinearHeading(new Vector2d(44,-50), 3.0/2*Math.PI)
//                    .waitSeconds(0.5)
//                    //Second Spike
//                    .strafeToLinearHeading(new Vector2d(48,-25), 3.0/2*Math.PI)
//                    .waitSeconds(0.5)
//                    //Human Player
//                    .strafeToLinearHeading(new Vector2d(47,-50), 3.0/2*Math.PI)
//                    .strafeToLinearHeading(new Vector2d(47,-46.5), 3.0/2*Math.PI)
//                    .strafeToLinearHeading(new Vector2d(44,-50), 3.0/2*Math.PI)
//                    .waitSeconds(0.5)
//                    //High Rung Position
//                    .strafeTo(new Vector2d(0,-35))
//                    .waitSeconds(1)
//                    .setReversed(false)
//                    //Third Spike
//                    .splineTo(new Vector2d(58,-25), 0)
//                    .waitSeconds(0.5)
//                    //Human Player
//                    .strafeToLinearHeading(new Vector2d(47,-50), 3.0/2*Math.PI)
//                    .strafeToLinearHeading(new Vector2d(47,-46.5), 3.0/2*Math.PI)
//                    .strafeToLinearHeading(new Vector2d(44,-50), 3.0/2*Math.PI)
//                    .waitSeconds(0.5)
//                    //High Rung
//                    .strafeTo(new Vector2d(0,-35))
//                    .waitSeconds(1)
//                    //Human Player
//                    .strafeToLinearHeading(new Vector2d(47,-50), 3.0/2*Math.PI)
//                    .strafeToLinearHeading(new Vector2d(47,-46.5), 3.0/2*Math.PI)
//                    .strafeToLinearHeading(new Vector2d(44,-50), 3.0/2*Math.PI)
//                    .waitSeconds(0.5)
//                    //High Rung
//                    .strafeTo(new Vector2d(0,-35))
//                    .waitSeconds(1)
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