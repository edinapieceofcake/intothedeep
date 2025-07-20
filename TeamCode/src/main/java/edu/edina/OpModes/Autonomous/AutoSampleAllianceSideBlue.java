package edu.edina.OpModes.Autonomous;

import static edu.edina.OpModes.Autonomous.AutoSampleAllianceSideRed.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Sample: Blue Alliance Side", group = "Main")
public class AutoSampleAllianceSideBlue extends AutoSample {
    @Override
    public void setup() {
        AutoSample.START_X = -RED_START_X;
        AutoSample.BASKET_X = -RED_BASKET_X;
        AutoSample.FIRST_SPIKE_X = -RED_FIRST_SPIKE_X;
        AutoSample.SECOND_SPIKE_X = -RED_SECOND_SPIKE_X;
        AutoSample.THIRD_SPIKE_X = -RED_THIRD_SPIKE_X;
        AutoSample.START_H = 180 - RED_START_H;
        AutoSample.BASKET_H = 180 - RED_BASKET_H;
        AutoSample.FIRST_SPIKE_H = 180 - RED_FIRST_SPIKE_H;
        AutoSample.SECOND_SPIKE_H = 180 - RED_SECOND_SPIKE_H;
        AutoSample.THIRD_SPIKE_H = 180 - RED_THIRD_SPIKE_H;

        setInitPose(START_X, START_Y, START_H);
        super.setup();
    }
}
