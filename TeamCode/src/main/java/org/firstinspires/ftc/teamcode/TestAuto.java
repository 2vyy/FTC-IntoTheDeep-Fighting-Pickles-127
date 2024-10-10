package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name="Test Auto")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub hub = new ControlHub();
        Pose2d initialPose = new Pose2d(-13, -58, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        hub.init(hardwareMap, initialPose);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
            .splineTo(new Vector2d(-13, -33), Math.toRadians(90));


        //pauses until START is pressed
        waitForStart();

        Action trajectoryActionChosen = traj1.build();

        Actions.runBlocking(
                //maybe put the build() in a seperate variable? idk :3
                new ParallelAction(
                        traj1.build(),
                        hub.arm.extendOut()
                )
        );



    }
}
