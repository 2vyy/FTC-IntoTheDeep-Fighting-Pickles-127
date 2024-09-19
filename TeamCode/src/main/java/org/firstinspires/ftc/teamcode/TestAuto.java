package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Auto")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //when INIT is pressed, this will run
        int num = 5;
        double dec = 12.5;
        //do all of your initializations in ControlHub.java to not repeat yourself throughout files

//        ControlHub hub = new ControlHub();
//        hub.init(hardwareMap);

        //pauses until START is pressed
        waitForStart();

        for(int i = 0; i < 5; i++) {
            logTelemetry("Doing Task # " + i);
            sleep(1000); // this completely freezes the robot for a second. this will
            // keep servos, motors, whatever systems running until the timer is over (bad idea)
            // if using roadrunner, refer to its actions system instead of this
        }

//        hub.openClaw();
//        if (hub.isClawOpen()) {
//            //this would be true
//        }
    }

    public void logTelemetry(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }
}
