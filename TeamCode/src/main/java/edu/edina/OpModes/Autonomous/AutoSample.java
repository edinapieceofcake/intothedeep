package edu.edina.OpModes.Autonomous;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;


import java.util.function.Supplier;

import edu.edina.Libraries.Actions.LazyAction;
import edu.edina.Libraries.Actions.LogAction;
import edu.edina.Libraries.Actions.WaitUntil;
import edu.edina.Libraries.PurePursuit.Path;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.WaitForTime;

public abstract class AutoSample extends AutoBase {
    public static boolean FLIP_X;
    public static double START_X = -40, START_Y = 61, START_H = 0;
    public static double BASKET_X = -35, BASKET_Y = 75, BASKET_H = 135, BASKET_MAX_SPEED = 20, BASKET_RADIUS = 5;
    public static double FIRST_SPIKE_X = -22, FIRST_SPIKE_Y = 70, FIRST_SPIKE_H = 180, FIRST_SPIKE_MAX_SPEED = 20, FIRST_SPIKE_RADIUS = 4;
    public static double SECOND_SPIKE_X = -22, SECOND_SPIKE_Y = 74, SECOND_SPIKE_H = 180, SECOND_SPIKE_MAX_SPEED = 20, SECOND_SPIKE_RADIUS = 4;
    public static double THIRD_SPIKE_X = -22, THIRD_SPIKE_Y = 78, THIRD_SPIKE_H = 180, THIRD_SPIKE_MAX_SPEED = 20, THIRD_SPIKE_RADIUS = 4;

    @Override
    public void setup() {
        setInitPose(START_X, START_Y, START_H);
    }

    @Override
    public void initAuto() {
        hw.addPrimaryAction(makeScoreAction(this::makeFirstSpikeMarkAction));
    }

    public Action makeScoreAction(Supplier<Action> nextAction) {
        return new ParallelAction(
                new LogAction("score", "wallMode"),
                hw.makeWallModeAction(), // clean this up...
                new SequentialAction(
                        new WaitUntil("score/arm at wall", () -> hw.armAt(Arm.POS_ARM_WALL, 5)),
                        new SequentialAction(
                                new LogAction("score", "drive to basket"),
                                new LazyAction(() -> hw.sequencePath(score(), 3)),
                                new LogAction("score", "high basket"),
                                hw.makeHighBasketRearAction(),
                                new LogAction("score", "open claw"),
//                                new InstantAction(() -> hw.wristUp()),
                                new WaitForTime(100),
                                new InstantAction(() -> hw.openClaw()),
                                new WaitForTime(220),
                                new LogAction("score", "next action"),
                                new InstantAction(() -> hw.release()),
                                new InstantAction(() -> hw.addPrimaryAction(nextAction.get()))
                        )
                )
        );
    }

    public Action reset() {
        return new ParallelAction(
                hw.getArm().constantPower(-0.5),
                hw.getLift().moveAndHold(-1),
                hw.getExtension().moveAndHold(-1),
                new SequentialAction(
                        hw.addPath(new Path(new Vector2d[] {state.getCurrentPose().position}).withHeading(START_H)),
                        hw.makeBrakeAction(0)
                )
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
                                        hw.waitForGroundMode("firstSpikeMark"),
                                        new LogAction("firstSpikeMark", "ground intake"),
                                        new SequentialAction(
                                                hw.makeGroundIntakeModeAction(),
                                                new LogAction("firstSpikeMark", "done"),
                                                hw.release(),
                                                new LazyAction(() -> makeScoreAction(this::makeSecondSpikeMarkAction))
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
                                        hw.waitForGroundMode("secondSpikeMark"),
                                        new LogAction("secondSpikeMark", "ground intake"),
                                        hw.makeGroundIntakeModeAction(),
                                        new WaitForTime(100),
                                        new LogAction("secondSpikeMark", "done"),
                                        new LazyAction(() -> makeScoreAction(this::reset))
                                )
                        )
                )
        );
    }

    public Action makeThirdSpikeMarkAction() {
        Path path = new Path(new Vector2d[]{state.getCurrentPose().position, new Vector2d(THIRD_SPIKE_X, THIRD_SPIKE_Y)})
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
                                        hw.waitForGroundMode("thirdSpikeMark"),
                                        new LogAction("thirdSpikeMark", "ground intake"),
                                        hw.makeGroundIntakeModeAction(),
                                        new WaitForTime(100),
                                        new LogAction("secondSpikeMark", "done"),
                                        new LazyAction(() -> makeScoreAction(this::reset))
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