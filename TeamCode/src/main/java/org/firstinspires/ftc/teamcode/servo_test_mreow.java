package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo_test_mreow")
public class servo_test_mreow extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo swing = hardwareMap.get(Servo.class, "swing");
        swing.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                telemetry.addLine("moving up");
                swing.setPosition(RobotConstants.SWING_UP);
            } else if (gamepad1.b) {
                telemetry.addLine("moving down");
                swing.setPosition(RobotConstants.SWING_DOWN);
            }

            telemetry.update();
        }

    }
}
