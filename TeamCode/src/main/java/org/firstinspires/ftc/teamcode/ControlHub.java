package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ControlHub {
    private Servo claw;
    public void init(HardwareMap map) {
        claw = map.get(Servo.class, "claw"); // "claw" would be the name of the port in the Driver Hub app settings
        claw.setDirection(Servo.Direction.FORWARD);
    }

    public void openClaw() {
        claw.setPosition(0.75);
    }

    public boolean isClawOpen() {
        return (claw.getPosition() == 0.75);
    }
}
