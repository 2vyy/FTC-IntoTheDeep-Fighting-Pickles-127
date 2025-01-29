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

@Autonomous(name="left auto 🙏")
public class left_auto extends LinearOpMode {
    DcMotor slideMotor;

    PIDFController slidePID = new PIDFController(
            RobotConstants.SLIDE_kP,
            RobotConstants.SLIDE_kI,
            RobotConstants.SLIDE_kD,
            RobotConstants.SLIDE_kF
    );

    @Override
    public void runOpMode() throws InterruptedException {
        Servo swing = hardwareMap.get(Servo.class, "swing");
        swing.setDirection(Servo.Direction.FORWARD);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d initialPose = new Pose2d(-32, -61, Math.toRadians(180));
        Pose2d outerbasketPose = new Pose2d(-44, -44, Math.toRadians(235));
        Pose2d innerBasketPose = new Pose2d(-51, -51, Math.toRadians(235));
        Pose2d innerSamplePose = new Pose2d(-50, -44, Math.toRadians(90));
        Pose2d centerSamplePose = new Pose2d(-60.5, -44, Math.toRadians(90));
        Pose2d outerSamplePose = new Pose2d(-47, -26.5, Math.toRadians(180));
        Pose2d outerSampleSecondPose = new Pose2d(-52, -27.5, Math.toRadians(180));
        Pose2d parkingPose = new Pose2d(-36, 0, Math.toRadians(0));
        Pose2d finalPark = new Pose2d(-28, 0, Math.toRadians(0));

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder startToBasket = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(outerbasketPose.position, outerbasketPose.heading);

        TrajectoryActionBuilder innerBasketToOuterBasket = drive.actionBuilder(innerBasketPose)
                .strafeToLinearHeading(outerbasketPose.position, outerbasketPose.heading);

        TrajectoryActionBuilder outerBasketToInnerBasket = drive.actionBuilder(outerbasketPose)
                .strafeToLinearHeading(innerBasketPose.position, innerBasketPose.heading);

        TrajectoryActionBuilder basketToInnerSample = drive.actionBuilder(outerbasketPose)
                .strafeToLinearHeading(innerSamplePose.position, innerSamplePose.heading);

        TrajectoryActionBuilder innerSampleToBasket = drive.actionBuilder(innerSamplePose)
                .strafeToLinearHeading(outerbasketPose.position, outerbasketPose.heading);

        TrajectoryActionBuilder basketToCenterSample = drive.actionBuilder(outerbasketPose)
                .strafeToLinearHeading(centerSamplePose.position, centerSamplePose.heading);

        TrajectoryActionBuilder centerSampleToBasket = drive.actionBuilder(centerSamplePose)
                .strafeToLinearHeading(outerbasketPose.position, outerbasketPose.heading);

        TrajectoryActionBuilder basketToOuterSample = drive.actionBuilder(outerbasketPose)
                .strafeToLinearHeading(outerSamplePose.position, outerSamplePose.heading);

        TrajectoryActionBuilder outerSampleToBasket = drive.actionBuilder(outerSamplePose)
                .strafeToLinearHeading(outerbasketPose.position, outerbasketPose.heading);

        TrajectoryActionBuilder outerSampleToSecondPos = drive.actionBuilder(outerSamplePose)
                .strafeToLinearHeading(outerSampleSecondPose.position, outerSampleSecondPose.heading);

        TrajectoryActionBuilder secondPosToOuterSample = drive.actionBuilder(outerSampleSecondPose)
                .strafeToLinearHeading(outerSamplePose.position, outerSamplePose.heading);

        TrajectoryActionBuilder basketToPark = drive.actionBuilder(outerbasketPose)
                .strafeToLinearHeading(parkingPose.position, parkingPose.heading);

        TrajectoryActionBuilder parkToBar = drive.actionBuilder(parkingPose)
                .strafeToLinearHeading(finalPark.position, finalPark.heading);

        Action a_startToBasket = startToBasket.build();

        Action a_innerBasketToOuterBasket = innerBasketToOuterBasket.build();
        Action a_innerBasketToOuterBasketforInnerSample = innerBasketToOuterBasket.build();
        Action a_innerBasketToOuterBasketforCenterSample = innerBasketToOuterBasket.build();
        Action a_innerBasketToOuterBasketforOuterSample = innerBasketToOuterBasket.build();

        Action a_outerBasketToInnerBasket = outerBasketToInnerBasket.build();
        Action a_outerBasketToInnerBasketforInnerSample = outerBasketToInnerBasket.build();
        Action a_outerBasketToInnerBasketforCenterSample = outerBasketToInnerBasket.build();
        Action a_outerBasketToInnerBasketforOuterSample = outerBasketToInnerBasket.build();

        Action a_basketToInnerSample = basketToInnerSample.build();
        Action a_innerSampleToBasket = innerSampleToBasket.build();

        Action a_basketToCenterSample = basketToCenterSample.build();
        Action a_centerSampleToBasket = centerSampleToBasket.build();

        Action a_basketToOuterSample = basketToOuterSample.build();
        Action a_outerSampleToSecondPos = outerSampleToSecondPos.build();
        Action a_outerSampleToBasket = outerSampleToBasket.build();
        Action a_secondPosToOuterSample = secondPosToOuterSample.build();

        Action a_parkToBar = parkToBar.build();


        Action a_basketToPark = basketToPark.build();

        waitForStart();

        Actions.runBlocking(
            new SequentialAction(
                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                new ServoAction(swing, RobotConstants.SWING_UP),
                new SleepAction(0.5),
                new ParallelAction(
                    a_startToBasket,
                    slideToPosition(RobotConstants.SLIDE_HIGH_BASKET_POS)
                ),
                new SequentialAction( // drop off sample
                    a_outerBasketToInnerBasket,
                    new SleepAction(0.15),
                    new ServoAction(claw, RobotConstants.CLAW_OPEN),
                    new SleepAction(0.5),
                    a_innerBasketToOuterBasket
                ),

                new ParallelAction(
                    a_basketToInnerSample,
                    slideToPosition(RobotConstants.SLIDE_REST_POS)
                ),
                new SleepAction(0.25),
                new ServoAction(swing, RobotConstants.SWING_AUTO_SAMPLE),
                new SleepAction(0.25),
                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                new SleepAction(0.25),
                new ServoAction(swing, RobotConstants.SWING_UP),
                new SleepAction(0.25),
                new ParallelAction(
                        a_innerSampleToBasket,
                        slideToPosition(RobotConstants.SLIDE_HIGH_BASKET_POS)
                ),
                new SequentialAction( // drop off sample
                    a_outerBasketToInnerBasketforInnerSample,
                    new SleepAction(0.15),
                    new ServoAction(claw, RobotConstants.CLAW_OPEN),
                    new SleepAction(0.5),
                    a_innerBasketToOuterBasketforInnerSample
                ),

                new ParallelAction(
                        a_basketToCenterSample,
                        slideToPosition(RobotConstants.SLIDE_REST_POS)
                ),
                new ServoAction(swing, RobotConstants.SWING_AUTO_SAMPLE),
                new SleepAction(0.5),
                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                new SleepAction(0.5),
                new ServoAction(swing, RobotConstants.SWING_UP),
                new SleepAction(0.5),
                new ParallelAction(
                        a_centerSampleToBasket,
                        slideToPosition(RobotConstants.SLIDE_HIGH_BASKET_POS)
                ),
                new SequentialAction( // drop off sample
                        a_outerBasketToInnerBasketforCenterSample,
                        new SleepAction(0.15),
                        new ServoAction(claw, RobotConstants.CLAW_OPEN),
                        new SleepAction(0.5),
                        a_innerBasketToOuterBasketforCenterSample
                ),

                new ParallelAction(
                        a_basketToOuterSample,
                        slideToPosition(RobotConstants.SLIDE_REST_POS),
                        new SequentialAction(
                                new SleepAction(0.5),
                                new ServoAction(swing, RobotConstants.SWING_AUTO_SAMPLE)
                        )
                ),
                new SleepAction(0.25),
                a_outerSampleToSecondPos,
                new SleepAction(0.25),
                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                new SleepAction(0.25),
                a_secondPosToOuterSample,
                new ServoAction(swing, RobotConstants.SWING_UP),
                new SleepAction(0.5),
                new ParallelAction(
                        a_outerSampleToBasket,
                        slideToPosition(RobotConstants.SLIDE_HIGH_BASKET_POS)
                ),
                new SequentialAction( // drop off sample
                        a_outerBasketToInnerBasketforOuterSample,
                        new SleepAction(0.15),
                        new ServoAction(claw, RobotConstants.CLAW_OPEN),
                        new SleepAction(0.5),
                        a_innerBasketToOuterBasketforOuterSample
                ),

                new ParallelAction(
                        a_basketToPark,
                        slideToPosition(RobotConstants.SLIDE_REST_POS),
                        new SequentialAction(
                                new SleepAction(0.3),
                                new ServoAction(swing, RobotConstants.SWING_AUTO_PARK)
                        )
                ),
                a_parkToBar
            )
        );
    }

    public class MotorAction implements Action {
        int distance = 0;
        int distanceTolerance = 30;
        ElapsedTime time = new ElapsedTime();
        boolean cooked = false;

        public MotorAction(int dist) {
            distance = dist;
            cooked = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slideMotor.setPower(slidePID.calculate(distance, slideMotor.getCurrentPosition()));
            if(!cooked) {
                cooked = true;
                time.reset();
            }

            if((slideMotor.getCurrentPosition()>(distance-distanceTolerance) &&
                    slideMotor.getCurrentPosition()<(distance+distanceTolerance) || this.time.seconds()>3.0)) {
                slideMotor.setPower(RobotConstants.SLIDE_kF);
                return false;
            } else {
                return true;
            }
        }
    }

    public Action slideToPosition(int dist) { return new MotorAction(dist); }

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


