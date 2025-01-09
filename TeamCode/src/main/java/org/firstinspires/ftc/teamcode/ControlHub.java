package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ControlHub {
    MecanumDrive drive; //Roadrunner class, required for trajectories and stuff
    Slide slide;

    public void init(HardwareMap map, Pose2d initialPose) {
        slide = new Slide(map);
        drive = new MecanumDrive(map, initialPose);
    }
}
