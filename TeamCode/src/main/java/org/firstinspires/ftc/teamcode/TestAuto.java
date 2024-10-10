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
        Pose2d initialPose = new Pose2d(-13, -58, Math.toRadians(90)); //Position that this auto SHOULD be started at
        hub.init(hardwareMap, initialPose); //initialPose is needed for hub.drive to be created.

        // Using hub.drive.actionBuilder and the initialPose to make a roadrunner trajectory.
        TrajectoryActionBuilder traj1 = hub.drive.actionBuilder(initialPose)
            .splineTo(new Vector2d(-13, -33), Math.toRadians(90)); // Same code as in MeepMeep!

        //pauses until START is pressed
        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        traj1.build(), // .build() converts the Trajectory into an Action.
                        hub.arm.extendOut(),
                        new SequentialAction(
                                hub.arm.retractBack()
                        )
                )
        );
        
        // this might be better ??? dont look at this
        // Action trajectoryActionChosen = traj1.build();
    }
}
