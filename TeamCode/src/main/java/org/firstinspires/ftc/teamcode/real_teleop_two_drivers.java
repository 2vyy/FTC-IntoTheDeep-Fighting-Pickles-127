package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="real_teleop_two_drivers")
public class real_teleop_two_drivers extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public double powerModifier = 1.0;

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
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        swing.setDirection(Servo.Direction.FORWARD);

        //slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            motorAction();


            if(gamepad1.b) {
                telemetry.addLine("opening claw");
                claw.setPosition(RobotConstants.CLAW_CLOSE);
            } else if (gamepad1.a) {
                telemetry.addLine("closing claw");
                claw.setPosition(RobotConstants.CLAW_OPEN);
            }

            if(gamepad2.y) {
                telemetry.addLine("moving up");
                swing.setPosition(RobotConstants.SWING_UP);
            } else if (gamepad2.a) {
                telemetry.addLine("moving down");
                swing.setPosition(RobotConstants.SWING_DOWN);
            } else if (gamepad2.b) {
                telemetry.addLine("moving down");
                swing.setPosition(RobotConstants.SWING_VERY_DOWN);
            } else if(gamepad2.x) {
                telemetry.addLine("moving to specimen");
                swing.setPosition(RobotConstants.SWING_SPECIMEN);
            }

            if(gamepad2.dpad_up) {
                slideMotor.setPower(.5);
            } else if(gamepad2.dpad_down) {
                slideMotor.setPower(-.5);
            } else if (gamepad2.dpad_left) {
                slideMotor.setPower(-1);
            } else {
                slideMotor.setPower(RobotConstants.SLIDE_kF);
            }

            telemetry.addLine("SlideMotorCurrPos: "+slideMotor.getCurrentPosition());

            telemetry.update();
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
