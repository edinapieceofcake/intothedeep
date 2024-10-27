package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;

public class RobotHardware {
    /*
    Control Hub Portal
        Control Hub
            Motors
                0 - GoBILDA 5201 series - right_lift_motor (has right lift encoder)
                1 - GoBILDA 5201 series - front_encoder (is front odometry encoder)
                2 - GoBILDA 5201 series - left_back_drive (encoder port returns 0 and -1)
                3 - GoBILDA 5201 series - left_front_drive (has left odometry encoder)
            Servos
                0 - Axon Micro+ ServoE - wrist_left
                1 - GoBilda 5 turn - claw_servo
                2 - Axon Micro+ ServoF - wrist_right
            I2C`
                3 - Neopixel Driver - neopixel_driver
            Analog
                0 - Axon Micro+ ServoE - wrist_left_encoder
                1 - Axon Micro+ ServoF - wrist_right_encoder
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - left_lift_motor (has left lift encoder)
                1 - GoBILDA 5201 series - arm_motor (has through bore encoder)
                2 - GoBILDA 5201 series - right_front_drive (encoder port has bent pin)
                3 - GoBILDA 5201 series - right_back_drive (has right odometry encoder)
            Servos
                0 - CRServo Axon Mini+ - slide_servo
            Analog
                0 - Axon Mini+ Encoder - slide_encoder
    */

    public final DcMotorEx leftFrontDrive, rightFrontDrive, rightBackDrive, leftBackDrive;
    public final DcMotorEx liftMotorL, liftMotorR;
    public final DcMotorEx armMotor;
    public final CRServo slideServo;
    public final Servo claw, wristLeft, wristRight;
    public final AnalogInput slideEncoder, wristEncoderL, wristEncoderR;
    public final IMU imu;
    public final Telemetry telemetry;
    public ThreeDeadWheelLocalizer odometry;
    public final MecanumDrive drive;

    public RobotHardware(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        odometry = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorL = hardwareMap.get(DcMotorEx.class, "left_lift_motor");
        liftMotorR = hardwareMap.get(DcMotorEx.class, "right_lift_motor");
        liftMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        slideServo = hardwareMap.get(CRServo.class, "slide_servo");

        wristLeft = hardwareMap.get(Servo.class, "wrist_left");
        wristRight = hardwareMap.get(Servo.class, "wrist_right");
        claw = hardwareMap.get(Servo.class, "claw_servo");

        slideEncoder = hardwareMap.get(AnalogInput.class, "slide_encoder");
        wristEncoderL = hardwareMap.get(AnalogInput.class, "wrist_left_encoder");
        wristEncoderR = hardwareMap.get(AnalogInput.class, "wrist_right_encoder");
    }

    public double getSlideVoltage() {
        return slideEncoder.getVoltage();
    }

    public double getWristLVolt() {
        return wristEncoderL.getVoltage();
    }

    public double getWristRVolt() {
        return wristEncoderR.getVoltage();
    }

    public int getLeftLift() {
        return liftMotorL.getCurrentPosition();
    }

    public int getRightLift() {
        return liftMotorR.getCurrentPosition();
    }
}