package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name="TeleOpasdasd")
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //when INIT is pressed, this will run
        //ControlHub hub = new ControlHub();
        //hub.init(hardwareMap);

        //pauses until START is pressed
        waitForStart();

        //when START is pressed, this will run
        //teleop-specific: runs until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            armAction();
        }
    }

    public void armAction() {
        //these two if statements do the same thing, but the second one is cleaner
        if(gamepad1.a) {
            telemetry.addLine("A is pressed");
            telemetry.update();
        }
        if(gamepad1.a) {
            logTelemetry("A is pressed"); // this is cleaner
        }
    }

    public void logTelemetry(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }
}
