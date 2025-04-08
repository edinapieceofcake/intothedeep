package edu.edina.Libraries.Actions;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Conditions {
    public static class SpecimenPark implements Condition {
        private final OpMode opMode;

        public SpecimenPark(OpMode opMode) {
            this.opMode = opMode;
        }

        @Override
        public boolean run() {
            return opMode.gamepad1.a;
        }
    }
}
