package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ControlHub {

    Arm arm;
    MecanumDrive drive; //Roadrunner class, required for trajectories and stuff

    public void init(HardwareMap map, Pose2d initialPose) {
        drive = new MecanumDrive(map, initialPose);
        arm = new Arm(map);
    }
}
