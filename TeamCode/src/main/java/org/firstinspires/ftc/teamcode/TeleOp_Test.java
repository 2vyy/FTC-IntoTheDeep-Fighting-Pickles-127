package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="TeleOp_Test")

public class TeleOp_Test extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor extendArmMotor;
    public DcMotor baseArmMotor;
    public CRServo roller;
    public boolean initialized = false;
    public ElapsedTime timer = new ElapsedTime();

    ControlHub hub;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    double powerModifier = 1;

    @Override
    public void runOpMode() throws InterruptedException {
//        hub = new ControlHub();
        // initialPose is just here to make hub.init(HardwareMap map, Pose2d initialPose) work
        // unless you want to do roadrunner stuff in teleop :troll_face:
//        Pose2d initialPose = new Pose2d(-23, -10, Math.toRadians(0));
//        hub.init(hardwareMap, initialPose);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        extendArmMotor = hardwareMap.get(DcMotor.class, "extendArmMotor");
        roller = hardwareMap.get(CRServo.class, "roller");

        extendArmMotor.setTargetPosition(-130);
        extendArmMotor.setPower(0);
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        //pauses until START is pressed
        waitForStart();

        while(opModeIsActive()) {
            motorAction();

            telemetry.addLine("ExtendArm: "+extendArmMotor.getCurrentPosition());
            telemetry.update();
            armAction();



        }
    }

    public void armAction() {
        if(gamepad1.a) {
            extendArmMotor.setPower(.2);
            extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendArmMotor.setTargetPosition(-265);
        } else if (gamepad1.b) {
            extendArmMotor.setPower(.2);
            extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendArmMotor.setTargetPosition(-240);
        }

        if(gamepad1.dpad_down) {
            roller.setPower(1);
        } else if (gamepad1.dpad_up) {
            roller.setPower(-1);
        } else {
            roller.setPower(0);
        }
    }



    public void motorAction() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), powerModifier);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if(gamepad1.back) {
            powerModifier = 0.25;
        } else {
            powerModifier = 1;
        }

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }



    public void rrAction() {
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
