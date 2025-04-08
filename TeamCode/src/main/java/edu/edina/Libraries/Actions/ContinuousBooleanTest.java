package edu.edina.Libraries.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ContinuousBooleanTest {
    private final double thresholdSeconds;
    private ElapsedTime t;

    public ContinuousBooleanTest(int thresholdMilliseconds) {
        thresholdSeconds = thresholdMilliseconds / 1000.0;
    }

    public boolean update(boolean lastValue) {
        if (lastValue) {
            if (t == null)
                t = new ElapsedTime();

            return t.seconds() > thresholdSeconds;
        } else {
            t = null;
            return false;
        }
    }
}