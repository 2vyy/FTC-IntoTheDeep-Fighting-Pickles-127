package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="left auto")
public class left_auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo swing = hardwareMap.get(Servo.class, "swing");
        swing.setDirection(Servo.Direction.FORWARD);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFController slidePID = new PIDFController(
                RobotConstants.SLIDE_kP,
                RobotConstants.SLIDE_kI,
                RobotConstants.SLIDE_kD,
                RobotConstants.SLIDE_kF
        );

        Pose2d initialPose = new Pose2d(-32, -61, Math.toRadians(180));
        Pose2d outerbasketPose = new Pose2d(-47, -47, Math.toRadians(235));
        Pose2d innerBasketPose = new Pose2d(-51, -51, Math.toRadians(235));
        Pose2d innerSamplePose = new Pose2d(-51.5, -39, Math.toRadians(90));
        Pose2d centerSamplePose = new Pose2d(-58.5, -39, Math.toRadians(90));
//        Pose2d outerSamplePose = new Pose2d(-58.5, -39, Math.toRadians(90));


        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

//        TrajectoryActionBuilder dropOffSample = drive.actionBuilder(initialPose)
//                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_CLOSE))
//                .waitSeconds(0.25)
//
//                .strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(235))
//                .waitSeconds(0.25)
//                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_UP))
//                .waitSeconds(1.25)
//                .stopAndAdd(new MotorAction(slideMotor, slidePID, RobotConstants.SLIDE_HIGH_BASKET_POS))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-51, -51), Math.toRadians(235))
//                .waitSeconds(0.5)
//                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_OPEN))
//                .waitSeconds(0.25)
//                .strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(235))
//                .waitSeconds(0.25)
//                .stopAndAdd(new MotorAction(slideMotor, slidePID, RobotConstants.SLIDE_REST_POS))
//                .waitSeconds(0.5)
//
//                .strafeToLinearHeading(new Vector2d(-51.5, -39), Math.toRadians(90))
//                .waitSeconds(0.25)
//                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_AUTO_SAMPLE))
//                .waitSeconds(1)
//                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_CLOSE))
//                .waitSeconds(0.25)
//                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_UP))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(235))
//                .waitSeconds(0.5)
//                .stopAndAdd(new MotorAction(slideMotor, slidePID, RobotConstants.SLIDE_HIGH_BASKET_POS))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-51, -51), Math.toRadians(235))
//                .waitSeconds(0.5)
//                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_OPEN))
//                .waitSeconds(0.25)
//                .strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(235))
//                .waitSeconds(0.25)
//                .stopAndAdd(new MotorAction(slideMotor, slidePID, RobotConstants.SLIDE_REST_POS))
//                .waitSeconds(0.5)
//
//                .strafeToLinearHeading(new Vector2d(-58.5, -39), Math.toRadians(90))
//                .waitSeconds(0.25)
//                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_AUTO_SAMPLE))
//                .waitSeconds(1);
//                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_CLOSE))
//                .waitSeconds(0.25)
//                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_UP))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(235))
//                .waitSeconds(0.5)
//                .stopAndAdd(new MotorAction(slideMotor, slidePID, RobotConstants.SLIDE_HIGH_BASKET_POS))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-51, -51), Math.toRadians(235))
//                .waitSeconds(0.5)
//                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_OPEN))
//                .waitSeconds(0.25)
//                .strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(235))
//                .waitSeconds(0.25)
//                .stopAndAdd(new MotorAction(slideMotor, slidePID, RobotConstants.SLIDE_REST_POS))
//                .waitSeconds(0.5);

//        Action a_dropOffSample = dropOffSample.build();

        TrajectoryActionBuilder startToBasket = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(outerbasketPose.position, outerbasketPose.heading);

//        TrajectoryActionBuilder innerBasketToOuterBasket = drive.actionBuilder(innerBasketPose)
//                .strafeToLinearHeading(outerbasketPose.position, outerbasketPose.heading);

        TrajectoryActionBuilder outerBasketToInnerBasket = drive.actionBuilder(outerbasketPose)
                .strafeToLinearHeading(innerBasketPose.position, innerBasketPose.heading);

        TrajectoryActionBuilder basketToInnerSample = drive.actionBuilder(innerBasketPose)
                .strafeToLinearHeading(innerSamplePose.position, innerSamplePose.heading);

        TrajectoryActionBuilder innerSampleToBasket = drive.actionBuilder(innerSamplePose)
                .strafeToLinearHeading(outerbasketPose.position, outerbasketPose.heading);

        TrajectoryActionBuilder basketToCenterSample = drive.actionBuilder(outerbasketPose)
                .strafeToLinearHeading(centerSamplePose.position, centerSamplePose.heading);

        TrajectoryActionBuilder centerSampleToBasket = drive.actionBuilder(centerSamplePose)
                .strafeToLinearHeading(outerbasketPose.position, outerbasketPose.heading);

        Action a_startToBasket = startToBasket.build();

//        Action a_innerBasketToOuterBasket = innerBasketToOuterBasket.build();
        Action a_outerBasketToInnerBasket = outerBasketToInnerBasket.build();

        Action a_basketToInnerSample = basketToInnerSample.build();
        Action a_innerSampleToBasket = innerSampleToBasket.build();

        Action a_basketToCenterSample = basketToCenterSample.build();
        Action a_centerSampleToBasket = centerSampleToBasket.build();

        waitForStart();

        Actions.runBlocking(
            new SequentialAction(
                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                new ServoAction(swing, RobotConstants.SWING_UP),
                new SleepAction(0.5),
                new ParallelAction(
                    a_startToBasket,
                    new MotorAction(slideMotor, slidePID, RobotConstants.SLIDE_HIGH_BASKET_POS)
                ),
                new SequentialAction( // drop off sample
                    a_outerBasketToInnerBasket,
                    new SleepAction(0.15),
                    new ServoAction(claw, RobotConstants.CLAW_OPEN),
                    new SleepAction(0.15)
                ),
                new ParallelAction(
                    a_basketToInnerSample,
                    new MotorAction(slideMotor, slidePID, RobotConstants.SLIDE_REST_POS)
                ),
                new SleepAction(0.5),
                new ServoAction(swing, RobotConstants.SWING_AUTO_SAMPLE),
                new SleepAction(2)
            )
        );
    }

    public class MotorAction implements Action {
        DcMotor slideMotor;
        PIDFController slidePIDF;
        double currPose;
        double position;
        ElapsedTime time = new ElapsedTime();

        public MotorAction(DcMotor m, PIDFController pidf, int p) {
            this.slideMotor = m;
            this.currPose = m.getCurrentPosition();
            this.slidePIDF = pidf;
            this.position = p;
            time.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currPose = slideMotor.getCurrentPosition();
            telemetryPacket.put("slidePos",currPose);
            telemetry.addLine(currPose+"");
            telemetry.update();
            this.slideMotor.setPower(
                    this.slidePIDF.calculate(
                            this.position,
                            this.currPose
                    )
            );

            //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough so return false and end the action.
            if(Math.abs(currPose-this.position)<RobotConstants.PID_ERROR_TOLERANCE || time.seconds()>7.0) {
                slideMotor.setPower(RobotConstants.SLIDE_kF);
                return false;
            }
            //if not, return true. Roadrunner will then repeat the action
            return true;
        }
    }

    public class ServoAction implements Action {
        Servo servo;
        double position;

        public ServoAction(Servo s, double p) {
            this.servo = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
        }
    }
}


