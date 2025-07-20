package edu.edina.OpModes.Autonomous;

import com.acmerobotics.roadrunner.*;

import edu.edina.Libraries.Actions.LazyAction;
import edu.edina.Libraries.Actions.WaitUntil;
import edu.edina.Libraries.PurePursuit.Path;

public abstract class AutoSpecimen extends AutoBase {
    public static double START_X = 0, START_Y = 0, START_H = 180;
    public static double BAR_X = 30, BAR_Y = 0, BAR_H = 180, BAR_RADIUS = 4, BAR_MAX_SPEED = 18, BAR_TARGET_SPEED = 0;
    public static double SCORE_X = 30, SCORE_Y = 5, SCORE_H = 90, SCORE_RADIUS = 4, SCORE_MAX_SPEED = 18, SCORE_TARGET_SPEED = 0;
    public static double EXTEND_AMT = 6;

    @Override
    public void setup() {
        setInitPose(START_X, START_Y, START_H);
    }

    @Override
    public void initAuto() {
        hw.addPrimaryAction(new SequentialAction(
                driveForward(),
                rotate(),
                score(),
                new LazyAction(this::reset)
        ));
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

    public Action driveForward() {
        return hw.sequencePath(new Path(new Vector2d[] {state.getCurrentPose().position, new Vector2d(BAR_X, BAR_Y)}).withHeading(BAR_H).withRadius(BAR_RADIUS).withMaxSpeed(BAR_MAX_SPEED).withTargetSpeed(BAR_TARGET_SPEED), 3);
    }

    public Action rotate() {
        return new ParallelAction(
                hw.sequencePath(new Path(new Vector2d[] {new Vector2d(BAR_X, BAR_Y), new Vector2d(SCORE_X, BAR_Y)}).withHeading(SCORE_H).withRadius(SCORE_RADIUS).withMaxSpeed(SCORE_MAX_SPEED).withTargetSpeed(SCORE_TARGET_SPEED), 3),
                new InstantAction(hw::highSpecimen)
        );
    }

    public Action score() {
        return new SequentialAction(
                hw.sequencePath(new Path(new Vector2d[] {new Vector2d(SCORE_X, BAR_Y), new Vector2d(SCORE_X, SCORE_Y)}).withHeading(SCORE_H).withRadius(SCORE_RADIUS).withMaxSpeed(SCORE_MAX_SPEED).withTargetSpeed(SCORE_TARGET_SPEED), 3),
                new ParallelAction(
                        hw.getExtension().moveAndHold(EXTEND_AMT),
                        new SequentialAction(
                                new WaitUntil("extension", () -> hw.getExtension().at(EXTEND_AMT, 1)),
                                new InstantAction(() -> hw.openClaw())
                        )
                )
        );
    }
}
