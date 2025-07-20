package edu.edina.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Specimen: Right", group = "Main")
public class AutoSpecimenScoreRight extends AutoSpecimen {
    @Override
    public void setup() {
        AutoSpecimen.START_X = -AutoSpecimenScoreLeft.START_X;
        AutoSpecimen.START_H = 180 - AutoSpecimenScoreLeft.START_H;
        AutoSpecimen.BAR_X = -AutoSpecimenScoreLeft.BAR_X;
        AutoSpecimen.BAR_H = 180 - AutoSpecimenScoreLeft.BAR_H;
        AutoSpecimen.SCORE_X = -AutoSpecimenScoreLeft.SCORE_X;
        AutoSpecimen.SCORE_H = 180 - AutoSpecimenScoreLeft.SCORE_H;

        super.setup();
    }
}
