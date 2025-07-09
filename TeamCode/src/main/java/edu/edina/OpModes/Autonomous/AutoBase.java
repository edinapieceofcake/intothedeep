package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.RobotHardwareChicago;
import edu.edina.Libraries.Robot.RobotState;

public abstract class AutoBase extends LinearOpMode {
    protected RobotHardwareChicago hw;
    private Pose2d initPose;
    protected Drivetrain dt;
    protected RobotState state;

    public AutoBase(double x, double y, double deg) {
        initPose = new Pose2d(x, y, Math.toRadians(deg));
    }

    public abstract void initAuto();

    @Override
    public void runOpMode() {
        this.hw = new RobotHardwareChicago(hardwareMap, initPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.dt = hw.getDrivetrain();
        this.state = hw.getRobotState();

        while(opModeInInit()){
            hw.initUpdate(telemetry);
            telemetry.update();
        }

        initAuto();

        while (opModeIsActive()) {
            hw.update(telemetry);
            telemetry.update();
        }
    }
}