package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private IMU imu;

    private DcMotor slideMotor;

    private Servo swing;

    private Servo claw;

    private PIDFController slidePIDF;
    private boolean is_PIDF_Active = false;
    private int targetPosition;

    private double powerModifier = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        swing = hardwareMap.get(Servo.class, "swing");

        claw = hardwareMap.get(Servo.class, "claw");

        slidePIDF = new PIDFController(
                RobotConstants.SLIDE_kP,
                RobotConstants.SLIDE_kI,
                RobotConstants.SLIDE_kD,
                RobotConstants.SLIDE_kF
        );

        waitForStart();

        while (opModeIsActive()) {
            drivetrainAction();
            slideAction();
            armAction();
        }
    }

    public void drivetrainAction() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.back) {
            imu.resetYaw();

            telemetry.addLine("resetting heading");
            telemetry.update();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

//        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

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

    public void slideAction() {
        if(gamepad1.dpad_up && gamepad1.left_bumper) {
            is_PIDF_Active = true;
            targetPosition = RobotConstants.SLIDE_HIGH_BASKET_POS;
            swing.setPosition(RobotConstants.SWING_UP);
        } else if(gamepad1.dpad_down && gamepad1.left_bumper) {
            is_PIDF_Active = true;
            targetPosition = RobotConstants.SLIDE_REST_POS;
            swing.setPosition(RobotConstants.SWING_SPECIMEN);
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

    public void armAction() {
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
    }
}
