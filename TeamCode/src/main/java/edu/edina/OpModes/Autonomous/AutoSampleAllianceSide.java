package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.edina.Libraries.Actions.DriveForwardToDistanceAction;
import edu.edina.Libraries.Actions.SampleAlignAction;
import edu.edina.Libraries.PurePursuit.Path;
import edu.edina.Libraries.PurePursuit.PurePursuitAction;

@Config
@Autonomous
public class AutoSampleAllianceSide extends AutoBase {
    public static double START_X = -40, START_Y = -61, START_H = 180;
    public static double BASKET_X = -35, BASKET_Y = -85, BASKET_H = 225, BASKET_MAX_SPEED = 15, BASKET_RADIUS = 4;
    public static double FIRST_SPIKE_X, FIRST_SPIKE_Y, FIRST_SPIKE_H, FIRST_SPIKE_MAX_SPEED = 15, FIRST_SPIKE_RADIUS = 4;

    private Path firstScore;
    private Path firstSpikeMark;

    public AutoSampleAllianceSide() {
        super(START_X, START_Y, START_H);
        firstScore = new Path(new Vector2d[]{new Vector2d(START_X, START_Y), new Vector2d(BASKET_X, BASKET_Y)}, 0, BASKET_MAX_SPEED, BASKET_RADIUS, BASKET_H);
    }

    @Override
    public void initAuto() {
        hw.addAction(
                new SequentialAction(
//                        new PurePursuitAction(firstScore, dt, state),
//                        new InstantAction(() -> hw.highBasket()),
                        new SampleAlignAction(hw),
                        new DriveForwardToDistanceAction(state, hw, 2, 1)
                )
        );
    }
}