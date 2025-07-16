package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import java.util.function.Supplier;

import edu.edina.Libraries.Actions.LazyAction;
import edu.edina.Libraries.Actions.LogAction;
import edu.edina.Libraries.Actions.SampleAlignAction;
import edu.edina.Libraries.Actions.WaitUntil;
import edu.edina.Libraries.PurePursuit.Path;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.WaitForTime;

@Config
@Autonomous(name = "Sample: Alliance Side", group = "Main")
public class AutoSampleAllianceSide extends AutoBase {
    public static double START_X = -40, START_Y = 61, START_H = 0;
    public static double BASKET_X = -33, BASKET_Y = 80, BASKET_H = 135, BASKET_MAX_SPEED = 35, BASKET_RADIUS = 5;
    public static double FIRST_SPIKE_X = -23, FIRST_SPIKE_Y = 67, FIRST_SPIKE_H = 180, FIRST_SPIKE_MAX_SPEED = 35, FIRST_SPIKE_RADIUS = 4;
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

        hw.addPrimaryAction(makeScoreAction(this::makeFirstSpikeMarkAction));
    }

    public Action makeScoreAction(Supplier<Action> nextAction) {
        return new ParallelAction(
                new LogAction("score", "wallMode"),
                hw.makeWallModeAction(), // clean this up...
                new SequentialAction(
                        new WaitUntil(() -> hw.armAt(Arm.POS_ARM_WALL, 5)),
                        new SequentialAction(
                                new LogAction("score", "drive to basket"),
                                new LazyAction(() -> hw.sequencePath(score(), 3)),
                                new LogAction("score", "high basket"),
                                hw.makeHighBasketRearAction(),
                                new LogAction("score", "open claw"),
                                new InstantAction(() -> hw.wristUp()),
                                new WaitForTime(300),
                                new InstantAction(() -> hw.openClaw()),
                                new LogAction("score", "next action"),
                                new InstantAction(() -> hw.addPrimaryAction(nextAction.get()))
                        )
                )
        );
    }

    public Action makeScoreAction() {
        return new SequentialAction(
                new LogAction("score", "wallMode"),
                hw.makeWallModeAction(), // clean this up...
                new LogAction("score", "drive to basket"),
                hw.sequencePath(score(), 3),
                new LogAction("score", "high basket"),
                hw.makeHighBasketRearAction(),
                new LogAction("score", "open claw"),
                new InstantAction(() -> hw.openClaw())
        );
    }

    public Action makeStealSpikeMarkAction() {
        Path path = new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(FIRST_SPIKE_X, FIRST_SPIKE_Y), new Vector2d(OPP_FIRST_SPIKE_X - 10, (OPP_SECOND_SPIKE_Y + OPP_SECOND_SPIKE_Y) / 2.0)})
                .withMaxSpeed(OPP_FIRST_SPIKE_MAX_SPEED)
                .withRadius(FIRST_SPIKE_RADIUS)
                .withHeading(FIRST_SPIKE_H);

        return new ParallelAction(
                new LogAction("stealSample", "wallMode"),
                hw.makeWallModeAction(),
                new SequentialAction(
                        new LogAction("stealSample", "drive"),
                        hw.sequencePath(path, 4),
                        new ParallelAction(
                                new LogAction("stealSample", "groundHold"),
                                hw.makeGroundModeAction(),
                                new SequentialAction(
                                        hw.waitForGroundMode(),
                                        new SampleAlignAction(hw),
                                        new LogAction("stealSample", "drive"),
                                        new LazyAction(() -> {
                                            if (hw.getSampleLocation() != null) {
                                                return stealScore();
                                            } else {
                                                return noSteal();
                                            }
                                        }),
                                        new LazyAction(this::makeSecondSpikeMarkAction)
                                )
                        )
                )
        );
    }

    public Action noSteal() {
        return new SequentialAction(
                hw.sequencePath(new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(FIRST_SPIKE_X, FIRST_SPIKE_Y)}).withTargetSpeed(4).withHeading(FIRST_SPIKE_H).withMaxSpeed(FIRST_SPIKE_MAX_SPEED).withRadius(FIRST_SPIKE_RADIUS), 3),
                hw.sequencePath(new Path(new Vector2d[]{new Vector2d(FIRST_SPIKE_X, FIRST_SPIKE_Y), new Vector2d(FIRST_SPIKE_X - 10, FIRST_SPIKE_Y)}).withMaxSpeed(FIRST_SPIKE_MAX_SPEED).withHeading(FIRST_SPIKE_H), 3)
        );
    }

    public Action stealScore() {
        return new SequentialAction(
                hw.sequencePath(new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(OPP_FIRST_SPIKE_X, state.getCurrentPose().position.y)}).withMaxSpeed(OPP_FIRST_SPIKE_MAX_SPEED).withRadius(OPP_FIRST_SPIKE_RADIUS).withHeading(OPP_FIRST_SPIKE_H), 3),
                new LogAction("stealSample", "intake"),
                hw.makeGroundIntakeModeAction(),
                new WaitForTime(100),
                new LogAction("stealSample", "done"),
                new LazyAction(() -> new SequentialAction(
                        hw.sequencePath(new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(FIRST_SPIKE_X, FIRST_SPIKE_Y)}).withMaxSpeed(FIRST_SPIKE_MAX_SPEED).withRadius(FIRST_SPIKE_RADIUS).withHeading(FIRST_SPIKE_H).withTargetSpeed(6), 3),
                        new LazyAction(this::makeScoreAction)
                ))
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
                                        new SequentialAction(
                                                hw.makeGroundIntakeModeAction(),
                                                new LogAction("firstSpikeMark", "done"),
                                                hw.release(),
                                                new LazyAction(() -> makeScoreAction(this::makeStealSpikeMarkAction))
                                        )
                                )
                        )
                )
        );
    }

    public Action makeSecondSpikeMarkAction() {
        Path path = new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(SECOND_SPIKE_X, SECOND_SPIKE_Y)})
                .withMaxSpeed(SECOND_SPIKE_MAX_SPEED)
                .withRadius(SECOND_SPIKE_RADIUS)
                .withHeading(SECOND_SPIKE_H);

        return new ParallelAction(
                new LogAction("secondSpikeMark", "wall mode"),
                hw.makeWallModeAction(),
                new SequentialAction(
                        new LogAction("secondSpikeMark", "drive"),
                        hw.sequencePath(path, 3),
                        new ParallelAction(
                                new LogAction("secondSpikeMark", "ground hold"),
                                hw.makeGroundModeAction(),
                                new SequentialAction(
                                        hw.waitForGroundMode(),
                                        new LogAction("secondSpikeMark", "ground intake"),
                                        hw.makeGroundIntakeModeAction(),
                                        new WaitForTime(100),
                                        new LogAction("secondSpikeMark", "done"),
                                        new LazyAction(() -> makeScoreAction(this::makeThirdSpikeMarkAction))
                                )
                        )
                )
        );
    }

    public Action makeThirdSpikeMarkAction() {
        Path path = new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(SECOND_SPIKE_X, SECOND_SPIKE_Y)})
                .withMaxSpeed(THIRD_SPIKE_MAX_SPEED)
                .withRadius(THIRD_SPIKE_RADIUS)
                .withHeading(THIRD_SPIKE_H);

        return new ParallelAction(
                new LogAction("thirdSpikeMark", "wall mode"),
                hw.makeWallModeAction(),
                new SequentialAction(
                        new LogAction("thirdSpikeMark", "drive"),
                        hw.sequencePath(path, 3),
                        new ParallelAction(
                                new LogAction("thirdSpikeMark", "ground hold"),
                                hw.makeGroundModeAction(),
                                new SequentialAction(
                                        hw.waitForGroundMode(),
                                        new LogAction("thirdSpikeMark", "ground intake"),
                                        hw.makeGroundIntakeModeAction(),
                                        new WaitForTime(100),
                                        new LogAction("secondSpikeMark", "done"),
                                        new LazyAction(this::makeScoreAction)
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
}