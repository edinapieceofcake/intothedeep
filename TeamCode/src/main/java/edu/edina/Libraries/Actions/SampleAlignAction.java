package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.Robot.RobotHardwareChicago;
import edu.edina.Libraries.Robot.SampleLocation;

@Config
public class SampleAlignAction implements Action {
    private static String TAG = "SampleAlignAction";

    private RobotHardwareChicago hw;

    public static double POWER = 0.32;
    public static double LATERAL_TOL = 5;

    public SampleAlignAction(RobotHardwareChicago hw) {
        this.hw = hw;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        SampleLocation s = hw.getSampleLocation();
        if (s != null) {
            double loc = s.getRelativeScreenLocation();

            //if cant find use timeout?

            if (Math.abs(loc) > LATERAL_TOL) {
                double p = POWER * Math.signum(loc);
                RobotLog.ii(TAG, "drive power %.2f", p);
                hw.getDrivetrain().update(0, p, 0);
            } else {
                RobotLog.ii(TAG, "drive stop");
                hw.getDrivetrain().stop();
                return false;
            }
        }
        return true;
    }
}
