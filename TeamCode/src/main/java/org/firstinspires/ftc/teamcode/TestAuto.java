package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Test Auto")
public class TestAuto extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public com.qualcomm.robotcore.hardware.DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor extendArmMotor;
    public DcMotor baseArmMotor;
    public CRServo roller;
    public boolean initialized = false;
    public ElapsedTime timer = new ElapsedTime();
    private final double MOTOR_TICKS_PER_REV = 288;
    private final double MOTOR_CIRCUMFERENCE = 3.14;
    private final double MOTOR_TICKS_PER_INCH = MOTOR_TICKS_PER_REV/MOTOR_CIRCUMFERENCE;
    private final double TICKS_PER_REV = 28 * 20;
    private final double CIRCUMFERENCE = 3.25;
    private final double TICKS_PER_INCH = 1200.0 / 24.0;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

//        baseArmMotor = hardwareMap.get(DcMotor.class, "baseArmMotor");
        extendArmMotor = hardwareMap.get(DcMotor.class, "extendArmMotor");
//        roller = hardwareMap.get(CRServo.class, "roller");

        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmMotor.setTargetPosition(0);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        drive(24);
        armForward();
        while(extendArmMotor.isBusy()) {

        }




    }

    public void armForward() {
        if(!initialized) {
            extendArmMotor.setPower(0.1);
            extendArmMotor.setTargetPosition(-130);
            if(extendArmMotor.getCurrentPosition()<-140) {
                initialized = true;
                extendArmMotor.setPower(0);
            }
        }
    }

    public void drive(double inches) { // a little off, might need increase
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setPower(0.25);
        frontRightMotor.setPower(0.25);
        backRightMotor.setPower(0.25);
        backLeftMotor.setPower(0.25);

        frontLeftMotor.setTargetPosition((int) (inches*TICKS_PER_INCH));
        frontRightMotor.setTargetPosition((int) (inches*TICKS_PER_INCH));
        backLeftMotor.setTargetPosition((int) (inches*TICKS_PER_INCH));
        backRightMotor.setTargetPosition((int) (inches*TICKS_PER_INCH));

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() && (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backRightMotor.isBusy() || backLeftMotor.isBusy())) {
            telemetry.addLine(frontLeftMotor.getCurrentPosition()+"");
            telemetry.addLine(frontLeftMotor.getTargetPosition()+"");
            telemetry.update();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

    }
}
