package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TrackingServo {
    private final Servo s;
    private final double fullRangeTime;

    private double prevPos, tgtPos;
    private ElapsedTime t;

    public TrackingServo(Servo s, double fullRangeTime) {
        this.s = s;
        this.fullRangeTime = fullRangeTime;
    }

    public void setPosition(double tgtPos) {
        s.setPosition(tgtPos);

        if (t == null) { // first time
            prevPos = tgtPos;
        } else {
            prevPos = getEstimatedPosition();
        }

        this.tgtPos = tgtPos;
        t = new ElapsedTime();
    }

    public double getEstimatedPosition() {
        if (t == null || tgtPos == prevPos)
            return tgtPos;

        double elapsedTime = t.seconds();
        double estTravelTime = Math.abs(tgtPos - prevPos) * fullRangeTime;
        double x = Math.min(elapsedTime / estTravelTime, 1);
        return (tgtPos - prevPos) * x + prevPos;
    }
}
