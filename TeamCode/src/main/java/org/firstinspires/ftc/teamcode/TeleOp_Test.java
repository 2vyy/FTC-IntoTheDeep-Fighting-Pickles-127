package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="TeleOp_Test")

public class TeleOp_Test extends LinearOpMode {


    ControlHub hub;

    @Override
    public void runOpMode() throws InterruptedException {
        hub = new ControlHub();
        // initialPose is just here to make hub.init(HardwareMap map, Pose2d initialPose) work
        // unless you want to do roadrunner stuff in teleop :troll_face:
        Pose2d initialPose = new Pose2d(-23, -10, Math.toRadians(0));
        hub.init(hardwareMap, initialPose);

        //pauses until START is pressed
        waitForStart();

        while(opModeIsActive()) {
            hub.motorAction(gamepad1);
            hub.arm.armAction(gamepad1);



        }
    }


}
