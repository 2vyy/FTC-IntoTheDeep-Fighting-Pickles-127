package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="TeleOp_Test")

public class TeleOp_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub hub = new ControlHub();
        Pose2d initialPose = new Pose2d(-23, -10, Math.toRadians(0);
        hub.init(hardwareMap, initialPose);

        //pauses until START is pressed
        waitForStart();

        while(opModeIsActive()) {
            hub.motorAction(gamepad1);
    }
}}
