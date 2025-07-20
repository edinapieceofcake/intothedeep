package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.Robot.RobotHardwareChicago;

@Config
@Autonomous
public class AutoPark extends LinearOpMode {
    public static double LATERAL = -0.5;
    public static double TIME_DRIVING = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);

        waitForStart();

        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive()) {
            hw.update(telemetry);
            hw.getDrivetrain().update(0, LATERAL, 0);

            if (t.seconds() > TIME_DRIVING) {
                terminateOpModeNow();
            }
        }
    }
}
