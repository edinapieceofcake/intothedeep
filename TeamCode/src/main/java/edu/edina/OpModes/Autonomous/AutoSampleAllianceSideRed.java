package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Sample: Red Alliance Side", group = "Main")
public class AutoSampleAllianceSideRed extends AutoSample {
    public static double RED_START_X = -40;
    public static double RED_BASKET_X = -33;
    public static double RED_FIRST_SPIKE_X = -23;
    public static double RED_SECOND_SPIKE_X = -26;
    public static double RED_THIRD_SPIKE_X = -26;
    public static double RED_OPP_FIRST_SPIKE_X = 18;
    public static double RED_OPP_SECOND_SPIKE_X = 18;
    public static double RED_OPP_THIRD_SPIKE_X = 18;

    public static double RED_START_H = 0;
    public static double RED_BASKET_H = 135;
    public static double RED_FIRST_SPIKE_H = 180;
    public static double RED_SECOND_SPIKE_H = 180;
    public static double RED_THIRD_SPIKE_H = 180;
    public static double RED_OPP_FIRST_SPIKE_H = 180;
    public static double RED_OPP_SECOND_SPIKE_H = 180;
    public static double RED_OPP_THIRD_SPIKE_H = 180;

    @Override
    public void setup() {
        AutoSample.START_X = RED_START_X;
        AutoSample.BASKET_X = RED_BASKET_X;
        AutoSample.FIRST_SPIKE_X = RED_FIRST_SPIKE_X;
        AutoSample.SECOND_SPIKE_X = RED_SECOND_SPIKE_X;
        AutoSample.THIRD_SPIKE_X = RED_THIRD_SPIKE_X;

        AutoSample.START_H = RED_START_H;
        AutoSample.BASKET_H = RED_BASKET_H;
        AutoSample.FIRST_SPIKE_H = RED_FIRST_SPIKE_H;
        AutoSample.SECOND_SPIKE_H = RED_SECOND_SPIKE_H;
        AutoSample.THIRD_SPIKE_H = RED_THIRD_SPIKE_H;

        setInitPose(RED_START_X, START_Y, RED_START_H);
    }
}

