package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="TeleOp_Test")

public class TeleOp_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub hub = new ControlHub();
        hub.init(hardwareMap);

        //pauses until START is pressed
        waitForStart();

        while(opModeIsActive()) {
            hub.motorAction(gamepad1);
    }
}}
