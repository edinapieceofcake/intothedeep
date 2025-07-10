package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.edina.Libraries.Actions.DriveForwardToDistanceAction;
import edu.edina.Libraries.Actions.SampleAlignAction;
import edu.edina.Libraries.Actions.WaitUntil;
import edu.edina.Libraries.PurePursuit.Path;
import edu.edina.Libraries.PurePursuit.PurePursuitAction;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.Extension;
import edu.edina.Libraries.Robot.Lift;
import edu.edina.Libraries.Robot.WaitForTime;

@Config
@Autonomous
public class AutoSampleAllianceSide extends AutoBase {
    public static double START_X = -40, START_Y = 61, START_H = 0;
    public static double BASKET_X = -35, BASKET_Y = 85, BASKET_H = -45, BASKET_MAX_SPEED = 24, BASKET_RADIUS = 5;
    public static double FIRST_SPIKE_X = 16, FIRST_SPIKE_Y = 73, FIRST_SPIKE_H = 0, FIRST_SPIKE_MAX_SPEED = 24, FIRST_SPIKE_RADIUS = 5;

    private Path firstScore;
    private Path firstSpikeMark;

    public AutoSampleAllianceSide() {
        super(START_X, START_Y, START_H);
        firstScore = new Path(new Vector2d[]{new Vector2d(START_X, START_Y), new Vector2d(BASKET_X, BASKET_Y)})
                .withMaxSpeed(BASKET_MAX_SPEED)
                .withRadius(BASKET_RADIUS)
                .withName("path-1.csv")
                .withHeading(BASKET_H);
        firstSpikeMark = new Path(new Vector2d[] {new Vector2d(START_X, START_Y), new Vector2d(FIRST_SPIKE_X, FIRST_SPIKE_Y)})
                .withMaxSpeed(FIRST_SPIKE_MAX_SPEED)
                .withRadius(FIRST_SPIKE_RADIUS)
                .withHeading(FIRST_SPIKE_H);
    }

    @Override
    public void initAuto() {
        hw.addAction(
                new SequentialAction(
                        new InstantAction(() -> hw.closeClaw()),
                        new PurePursuitAction(firstScore, dt, state),
                        new InstantAction(() -> hw.brake()),
                        new InstantAction(() -> hw.highBasketRear()),
                        new WaitUntil(() -> state.getLiftPos() > Lift.POS_HIGH_BASKET - 2 && state.getExtensionPos() > Extension.POS_HIGH_BASKET - 1),
                        new WaitForTime(50),
                        new InstantAction(() -> hw.openClaw()),
                        new WaitForTime(50),
                        new ParallelAction(
                                new InstantAction(() -> hw.wallMode()),
                                new PurePursuitAction(firstSpikeMark, dt, state)
                        ),
                        new SampleAlignAction(hw),
                        new DriveForwardToDistanceAction(state, hw, 2, 1)
                )
        );
    }
}