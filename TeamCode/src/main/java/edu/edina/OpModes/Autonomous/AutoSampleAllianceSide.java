package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.edina.Libraries.Actions.LazyAction;
import edu.edina.Libraries.Actions.LogAction;
import edu.edina.Libraries.PurePursuit.Path;
import edu.edina.Libraries.Robot.WaitForTime;

@Config
@Autonomous(name = "Sample: Alliance Side", group = "Main")
public class AutoSampleAllianceSide extends AutoBase {
    public static double START_X = -40, START_Y = 61, START_H = 0;
    public static double BASKET_X = -37, BASKET_Y = 87, BASKET_H = 135, BASKET_MAX_SPEED = 35, BASKET_RADIUS = 5;
    public static double FIRST_SPIKE_X = -26, FIRST_SPIKE_Y = 76, FIRST_SPIKE_H = 180, FIRST_SPIKE_MAX_SPEED = 35, FIRST_SPIKE_RADIUS = 4;
    public static double SECOND_SPIKE_X = -26, SECOND_SPIKE_Y = 85, SECOND_SPIKE_H = 180, SECOND_SPIKE_MAX_SPEED = 24, SECOND_SPIKE_RADIUS = 5;
    public static double THIRD_SPIKE_X = -26, THIRD_SPIKE_Y = 88, THIRD_SPIKE_H = 180, THIRD_SPIKE_MAX_SPEED = 24, THIRD_SPIKE_RADIUS = 5;
    public static double OPP_FIRST_SPIKE_X = 18, OPP_FIRST_SPIKE_Y = 76, OPP_FIRST_SPIKE_H = 180, OPP_FIRST_SPIKE_MAX_SPEED = 24, OPP_FIRST_SPIKE_RADIUS = 5;
    public static double OPP_SECOND_SPIKE_X = 18, OPP_SECOND_SPIKE_Y = 85, OPP_SECOND_SPIKE_H = 180, OPP_SECOND_SPIKE_MAX_SPEED = 24, OPP_SECOND_SPIKE_RADIUS = 5;
    public static double OPP_THIRD_SPIKE_X = 18, OPP_THIRD_SPIKE_Y = 88, OPP_THIRD_SPIKE_H = 180, OPP_THIRD_SPIKE_MAX_SPEED = 24, OPP_THIRD_SPIKE_RADIUS = 5;

    public AutoSampleAllianceSide() {
        super(START_X, START_Y, START_H);
    }

    @Override
    public void initAuto() {
        hw.enableYellow();

        hw.addAction(makeScoreAction());
    }

    public Action makeScoreAction() {
        return new SequentialAction(
                new LogAction("score", "wallMode"),
                new InstantAction(hw::wallMode), // clean this up...
                new LogAction("score", "drive to basket"),
                hw.sequencePath(score(), 3),
                new LogAction("score", "high basket"),
                hw.makeHighBasketRearAction(),
                new LogAction("score", "open claw"),
                new InstantAction(() -> hw.openClaw()),
                new LogAction("score", "next action"),
                new LazyAction(this::makeFirstSpikeMarkAction)
        );
    }

    public Action makeFirstSpikeMarkAction() {
        Path path = new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(FIRST_SPIKE_X, FIRST_SPIKE_Y)})
                .withMaxSpeed(FIRST_SPIKE_MAX_SPEED)
                .withRadius(FIRST_SPIKE_RADIUS)
                .withHeading(FIRST_SPIKE_H);

        return new ParallelAction(
                new LogAction("firstSpikeMark", "wall mode"),
                hw.makeWallModeAction(),
                new SequentialAction(
                        new LogAction("firstSpikeMark", "drive"),
                        hw.sequencePath(path, 3),
                        new ParallelAction(
                                new LogAction("firstSpikeMark", "ground hold"),
                                hw.makeGroundModeAction(),
                                new SequentialAction(
                                        hw.waitForGroundMode(),
                                        new LogAction("firstSpikeMark", "ground intake"),
                                        hw.makeGroundIntakeModeAction(),
                                        new WaitForTime(100),
                                        new LogAction("firstSpikeMark", "done")
                                )
                        )
                )
        );
    }

    public Path score() {
        return new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(BASKET_X, BASKET_Y)})
                .withMaxSpeed(BASKET_MAX_SPEED)
                .withRadius(BASKET_RADIUS)
                .withHeading(BASKET_H);
    }

    public Path firstSpikeMark() {
        return new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(FIRST_SPIKE_X, FIRST_SPIKE_Y)})
                .withMaxSpeed(FIRST_SPIKE_MAX_SPEED)
                .withRadius(FIRST_SPIKE_RADIUS)
                .withHeading(FIRST_SPIKE_H);
    }

    public Path secondSpikeMark() {
        return new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(SECOND_SPIKE_X, SECOND_SPIKE_Y)})
                .withMaxSpeed(SECOND_SPIKE_MAX_SPEED)
                .withRadius(SECOND_SPIKE_RADIUS)
                .withHeading(SECOND_SPIKE_H);
    }

    public Path thirdSpikeMark() {
        return new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(THIRD_SPIKE_X, THIRD_SPIKE_Y)})
                .withMaxSpeed(THIRD_SPIKE_MAX_SPEED)
                .withRadius(THIRD_SPIKE_RADIUS)
                .withHeading(THIRD_SPIKE_H);
    }

    public Path oppFirstSpikeMark() {
        return new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(OPP_FIRST_SPIKE_X, OPP_FIRST_SPIKE_Y)})
                .withMaxSpeed(OPP_FIRST_SPIKE_MAX_SPEED)
                .withRadius(OPP_FIRST_SPIKE_RADIUS)
                .withHeading(OPP_FIRST_SPIKE_H);
    }

    public Path oppSecondSpikeMark() {
        return new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(OPP_SECOND_SPIKE_X, OPP_SECOND_SPIKE_Y)})
                .withMaxSpeed(OPP_SECOND_SPIKE_MAX_SPEED)
                .withRadius(OPP_SECOND_SPIKE_RADIUS)
                .withHeading(OPP_SECOND_SPIKE_H);
    }

    public Path oppThirdSpikeMark() {
        return new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(OPP_THIRD_SPIKE_X, OPP_THIRD_SPIKE_Y)})
                .withMaxSpeed(OPP_THIRD_SPIKE_MAX_SPEED)
                .withRadius(OPP_THIRD_SPIKE_RADIUS)
                .withHeading(OPP_THIRD_SPIKE_H);
    }
}