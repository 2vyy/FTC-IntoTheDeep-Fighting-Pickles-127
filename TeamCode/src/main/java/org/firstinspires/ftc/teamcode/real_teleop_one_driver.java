package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="A_real_teleop_one_driver")
public class real_teleop_one_driver extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private DcMotor slideMotor;

    private PIDFController slidePIDF;
    private boolean is_PIDF_Active = false;

    private double powerModifier = 1.0;
    private int targetPosition = RobotConstants.SLIDE_REST_POS;

// roadrunner stuff
//    FtcDashboard dash = FtcDashboard.getInstance();
//    List<Action> runningActions = new ArrayList<>();
//
//    Pose2d centerPos = new Pose2d(-28, 0, Math.toRadians(0));
//    Pose2d outerbasketPos = new Pose2d(-44, -44, Math.toRadians(225));
//
//    Pose2d finalPark = new Pose2d(-27, 0, Math.toRadians(0));
//    SparkFunOTOSDrive drive;

    ElapsedTime actionInputBuffer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight"); //*

        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo swing = hardwareMap.get(Servo.class, "swing");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        swing.setDirection(Servo.Direction.FORWARD);

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slidePIDF = new PIDFController(
                RobotConstants.SLIDE_kP,
                RobotConstants.SLIDE_kI,
                RobotConstants.SLIDE_kD,
                RobotConstants.SLIDE_kF
        );

        waitForStart();

        while(opModeIsActive()) {
            motorAction();
            slideAction();

            if(gamepad1.b) {
                telemetry.addLine("opening claw");
                claw.setPosition(RobotConstants.CLAW_CLOSE);
            } else if (gamepad1.a) {
                telemetry.addLine("closing claw");
                claw.setPosition(RobotConstants.CLAW_OPEN);
            }

            if(gamepad1.y) {
                telemetry.addLine("moving up");
                swing.setPosition(RobotConstants.SWING_UP);
            } else if (gamepad1.x) {
                telemetry.addLine("moving down");
                swing.setPosition(RobotConstants.SWING_DOWN);
            } else if (gamepad1.dpad_left) {
                telemetry.addLine("moving down");
                swing.setPosition(RobotConstants.SWING_VERY_DOWN);
            } else if (gamepad1.dpad_right) {
                telemetry.addLine("moving down");
                swing.setPosition(RobotConstants.SWING_AUTO_SPECIMEN);
            } else if(gamepad1.start) {
                telemetry.addLine("moving to specimen");
                swing.setPosition(RobotConstants.SWING_SPECIMEN);
            }

            telemetry.addLine(slideMotor.getCurrentPosition()+"");
            telemetry.update();
        }
    }

    public void slideAction() {
        if(gamepad1.dpad_up && gamepad1.left_bumper) {
            is_PIDF_Active = true;
            targetPosition = RobotConstants.SLIDE_HIGH_BASKET_POS;
        } else if(gamepad1.dpad_down && gamepad1.left_bumper) {
            is_PIDF_Active = true;
            targetPosition = RobotConstants.SLIDE_REST_POS;
        } else if(gamepad1.dpad_up) {
            is_PIDF_Active = false;
            slideMotor.setPower(1 * RobotConstants.SLIDE_SPEED);
        } else if(gamepad1.dpad_down) {
            is_PIDF_Active = false;
            slideMotor.setPower(-1 * RobotConstants.SLIDE_SPEED);
        }  else if (!is_PIDF_Active){
            slideMotor.setPower(RobotConstants.SLIDE_kF);
        }

        if(is_PIDF_Active) {
            if (Math.abs(slideMotor.getCurrentPosition() - targetPosition) < RobotConstants.PID_ERROR_TOLERANCE) {
                is_PIDF_Active = false;
            } else {
                slideMotor.setPower(
                    slidePIDF.calculate(
                        targetPosition,
                        slideMotor.getCurrentPosition()
                    )
                );
            }
        }
    }

    public void motorAction() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if(gamepad1.right_bumper) {
            powerModifier = 0.25;
        } else {
            powerModifier = 1;
        }

        frontLeftMotor.setPower(frontLeftPower * powerModifier);
        backLeftMotor.setPower(backLeftPower * powerModifier);
        frontRightMotor.setPower(frontRightPower * powerModifier);
        backRightMotor.setPower(backRightPower * powerModifier);
    }
}
