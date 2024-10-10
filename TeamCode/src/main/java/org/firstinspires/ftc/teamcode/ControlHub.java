package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ControlHub {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    private DcMotor slideMotor;
    private Servo basketFlipper;

    Arm arm;
    MecanumDrive drive;

    public void init(HardwareMap map, Pose2d initialPose) {
        frontLeftMotor = map.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = map.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = map.get(DcMotor.class, "backLeftMotor");
        backRightMotor = map.get(DcMotor.class, "backRightMotor");
        slideMotor = map.get(DcMotor.class, "slideMotor");
        basketFlipper = map.get(Servo.class, "basketFlipper");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        drive = new MecanumDrive(map, initialPose);
        arm = new Arm(map);
    }

    public void motorAction(Gamepad gamepad) {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);




    }

}
