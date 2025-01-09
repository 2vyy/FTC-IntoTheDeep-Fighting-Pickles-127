package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="test")
public class test extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight"); //*

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                frontRightMotor.setPower(-.1);
                telemetry.addLine(frontRightMotor.getCurrentPosition()+"");
            } else if(gamepad1.b) {
                frontRightMotor.setPower(.1);
                telemetry.addLine(frontRightMotor.getCurrentPosition()+"");
            } else {
                frontRightMotor.setPower(0);
                telemetry.addLine(frontRightMotor.getCurrentPosition()+"");
            }
            if(gamepad1.x) {
                backLeftMotor.setPower(.1);
                telemetry.addLine(backLeftMotor.getCurrentPosition()+"");
            }
            if(gamepad1.y) {
                backRightMotor.setPower(.1);
                telemetry.addLine(backRightMotor.getCurrentPosition()+"");
            }

            telemetry.update();
        }
    }
}
