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

        // Copy into opmode

        // COPY THIS LINE
        Pose2d beginPose = new Pose2d(-50, -60, Math.PI);
        // COPY THIS LINE

        myBot.runAction(myBot.getDrive().actionBuilder(beginPose)

                // COPY HERE
                .strafeToLinearHeading(new Vector2d(-48,-35), (double)1/2*Math.PI)
                .strafeToLinearHeading(new Vector2d(-60,-50), (double)1/2*Math.PI)
                .strafeToLinearHeading(new Vector2d(-58,-35), (double)1/2*Math.PI)
                .strafeToLinearHeading(new Vector2d(-60,-50), (double)1/2*Math.PI)
                .strafeToLinearHeading(new Vector2d(-57,-25), Math.PI)
                .strafeToLinearHeading(new Vector2d(-60,-50), (double)1/2*Math.PI)
                /*.setReversed(false)
                .splineTo(new Vector2d(-50,-35), Math.PI/2)
                .splineTo(new Vector2d(-60,-25), Math.PI)*/
                // STOP COPYING

                .build());

        // Only for meepmeep
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .addEntity(myBot)
                .start();
    }
}