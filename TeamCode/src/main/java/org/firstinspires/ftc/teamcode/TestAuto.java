package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Test Auto")
public class TestAuto extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public com.qualcomm.robotcore.hardware.DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor extendArmMotor;
    public DcMotor baseArmMotor;
    public CRServo roller;
    public boolean initialized = false;
    public ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

//        ControlHub hub = new ControlHub();
//        Pose2d initialPose = new Pose2d(-13, -58, Math.toRadians(90)); //Position that this auto SHOULD be started at
//        hub.init(hardwareMap, initialPose); //initialPose is needed for hub.drive to be created.
//
//        // Using hub.drive.actionBuilder and the initialPose to make a roadrunner trajectory.
//        TrajectoryActionBuilder traj1 = hub.drive.actionBuilder(initialPose)
//            .splineTo(new Vector2d(-13, -33), Math.toRadians(90)); // Same code as in MeepMeep!

        //pauses until START is pressed

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

//        baseArmMotor = hardwareMap.get(DcMotor.class, "baseArmMotor");
        extendArmMotor = hardwareMap.get(DcMotor.class, "extendArmMotor");
//        roller = hardwareMap.get(CRServo.class, "roller");

        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmMotor.setTargetPosition(0);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        armForward();
        while(extendArmMotor.isBusy()) {

        }

//        Actions.runBlocking(
//                //This follows traj1 and extends the armout THEN retracts the arm back
//                new SequentialAction(
//                        new ParallelAction(
//                                traj1.build(), // .build() converts the Trajectory into an Action.
//                                hub.arm.extendOut()
//                        ),
//                        hub.arm.retractBack(),
//                        new InstantAction(() -> hub.arm.stopRolling())
//                )
//        );
        
        // this might be better ??? dont look at this
        // Action trajectoryActionChosen = traj1.build();



    }

    public void armForward() {
        if(!initialized) {
            extendArmMotor.setPower(0.1);
            extendArmMotor.setTargetPosition(-130);
            if(extendArmMotor.getCurrentPosition()<-140) {
                initialized = true;
                extendArmMotor.setPower(0);
            }
        }
    }
}
