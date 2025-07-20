package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Specimen: Left", group = "Main")
public class AutoSpecimenScoreLeft extends AutoSpecimen {
    public static double START_X = 0, START_H = 0;
    public static double BAR_X = 30, BAR_H = 0;
    public static double SCORE_X = 30, SCORE_H = 90;

    @Override
    public void setup() {
        AutoSpecimen.START_X = START_X;
        AutoSpecimen.START_H = START_H;
        AutoSpecimen.BAR_X = BAR_X;
        AutoSpecimen.BAR_H = BAR_H;
        AutoSpecimen.SCORE_X = SCORE_X;
        AutoSpecimen.SCORE_H = SCORE_H;

        super.setup();
    }
}
