//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Autonomous(name="right auto ^_^")
//public class right_auto extends LinearOpMode {
//    DcMotor slideMotor;
//
//    PIDFController slidePID = new PIDFController(
//            RobotConstants.SLIDE_kP,
//            RobotConstants.SLIDE_kI,
//            RobotConstants.SLIDE_kD,
//            RobotConstants.SLIDE_kF
//    );
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Servo swing = hardwareMap.get(Servo.class, "swing");
//        swing.setDirection(Servo.Direction.FORWARD);
//
//        Servo claw = hardwareMap.get(Servo.class, "claw");
//        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
//
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        Pose2d initialPose = new Pose2d(5, -61, Math.toRadians(90));
//
//        Pose2d centerBarPose = new Pose2d(5, -33, Math.toRadians(90));
//        Pose2d centerBarSecondPose = new Pose2d(5, -40, Math.toRadians(90));
//
//        Vector2d innerSplinePose = new Vector2d(42, -35);
//        Vector2d innerSplineSecondPose = new Vector2d(45.5, -15);
//
//        Pose2d innerSamplePushPose = new Pose2d(45.5, -55, Math.toRadians(90));
//        Pose2d innerSamplePose = new Pose2d(45.5, -12, Math.toRadians(90));
//        Vector2d centerSplinePose = new Vector2d(56, -20);
//
//        Pose2d centerSamplePushPose = new Pose2d(56, -55, Math.toRadians(90));
//        Pose2d centerSamplePose = new Pose2d(56, -12, Math.toRadians(90));
//
//        Pose2d outerSamplePushPose = new Pose2d(62, -55, Math.toRadians(90));
//        Vector2d outerSplinePose = new Vector2d(62, -20);
//
//        Pose2d waitPose = new Pose2d(55, -40, Math.toRadians(90));
//        Pose2d parkPose = new Pose2d(55, -58, Math.toRadians(90));
//
//        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
//
//        TrajectoryActionBuilder goToCenter = drive.actionBuilder(initialPose)
//                .strafeToConstantHeading(centerBarPose.position);
//        Action a_goToCenter = goToCenter.build();
//
//        TrajectoryActionBuilder outFromCenter = drive.actionBuilder(centerBarPose)
//                .strafeToConstantHeading(centerBarSecondPose.position);
//        Action a_outFromCenter = outFromCenter.build();
//
//        TrajectoryActionBuilder centerToSpline1 = drive.actionBuilder(centerBarSecondPose)
//               .splineToConstantHeading(innerSplinePose, Math.toRadians(180));
//        Action a_centerToSpline1 = centerToSpline1.build();
//
//        waitForStart();
//
//        Actions.runBlocking(
//
//            new SequentialAction(
//                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
//                new ServoAction(swing, RobotConstants.SWING_UP), // change this
//                new SleepAction(1),
//                new ParallelAction(
//                    a_goToCenter,
//                    slideToPosition(RobotConstants.SLIDE_HIGH_BAR_POS)
//                ),
//                new SequentialAction( // drop off sample
//                    new SleepAction(0.35),
//                    new ServoAction(claw, RobotConstants.CLAW_OPEN),
//                    new SleepAction(0.35)
//                ),
//                new ParallelAction(
//                        a_outFromCenter,
//                        slideToPosition(RobotConstants.SLIDE_REST_POS)
//                ),
//                a_centerToSpline1
//
//
//
////
////                new ParallelAction(
////                    a_basketToInnerSample,
////                    slideToPosition(RobotConstants.SLIDE_REST_POS)
////                ),
////                new SleepAction(0.5),
////                new ServoAction(swing, RobotConstants.SWING_AUTO_SAMPLE),
////                new SleepAction(0.5),
////                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
////                new SleepAction(0.5),
////                new ServoAction(swing, RobotConstants.SWING_UP),
////                new SleepAction(0.5),
////                new ParallelAction(
////                        a_innerSampleToBasket,
////                        slideToPosition(RobotConstants.SLIDE_HIGH_BASKET_POS)
////                ),
////                new SequentialAction( // drop off sample
////                    a_outerBasketToInnerBasketforInnerSample,
////                    new SleepAction(0.15),
////                    new ServoAction(claw, RobotConstants.CLAW_OPEN),
////                    new SleepAction(0.15)
////                )
//
//
//            )
//        );
//    }
//
//    public class MotorAction implements Action {
//        int distance = 0;
//        int distanceTolerance = 30;
//
//        public MotorAction(int dist) {
//            distance = dist;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            slideMotor.setPower(slidePID.calculate(distance, slideMotor.getCurrentPosition()));
//
//            if(slideMotor.getCurrentPosition()>(distance-distanceTolerance) &&
//               slideMotor.getCurrentPosition()<(distance+distanceTolerance)) {
//                slideMotor.setPower(RobotConstants.SLIDE_kF);
//                return false;
//            } else {
//                return true;
//            }
//        }
//    }
//
//    public Action slideToPosition(int dist) { return new MotorAction(dist); }
//
//    public class ServoAction implements Action {
//        Servo servo;
//        double position;
//
//        public ServoAction(Servo s, double p) {
//            this.servo = s;
//            this.position = p;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            servo.setPosition(position);
//            return false;
//        }
//    }
//}
//
//

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

@Autonomous(name="right auto 🙏")
public class right_auto extends LinearOpMode {
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

        Pose2d initialPose = new Pose2d(1, -61, Math.toRadians(90));
        Pose2d centerBarPose = new Pose2d(1, -34, Math.toRadians(90));
        Pose2d centerBarSecondPose = new Pose2d(1, -50, Math.toRadians(90));

        Pose2d firstSplinePose = new Pose2d(30, -20, Math.toRadians(90));
        Pose2d secondSplinePose = new Pose2d(41 , -13, Math.toRadians(90));
        Pose2d firstCornerZone = new Pose2d(41, -52, Math.toRadians(90));

        Pose2d cornerToSecondSample = new Pose2d(41, -20, Math.toRadians(90));
        Pose2d secondSampleSpline = new Pose2d(50, -13, Math.toRadians(90));
        Pose2d secondCornerZone = new Pose2d(50, -52, Math.toRadians(90));

        Pose2d cornerToThirdSample = new Pose2d(50, -20, Math.toRadians(90));
        Pose2d thirdSampleSpline = new Pose2d(57, -13, Math.toRadians(90));
        Pose2d thirdCornerZone = new Pose2d(57, -52, Math.toRadians(90));
        Pose2d cornerZoneSecondPose = new Pose2d(57, -40, Math.toRadians(90));
        Pose2d finalPark = new Pose2d(57, -54, Math.toRadians(90));




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

        TrajectoryActionBuilder goToCenter = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(centerBarPose.position);
        Action a_goToCenter = goToCenter.build();

        TrajectoryActionBuilder outFromCenter = drive.actionBuilder(centerBarPose)
                .strafeToConstantHeading(centerBarSecondPose.position);
        Action a_outFromCenter = outFromCenter.build();

        TrajectoryActionBuilder barToInnerSample = drive.actionBuilder(centerBarSecondPose)
//                .strafeToLinearHeading(innerSamplePose.position, innerSamplePose.heading);
                .setReversed(true)
                .splineToConstantHeading(firstSplinePose.position, Math.toRadians(90))
                .setReversed(false)
                .splineToConstantHeading(secondSplinePose.position, Math.toRadians(0));
        Action a_barToInnerSample = barToInnerSample.build();

        TrajectoryActionBuilder innerSampleToCorner = drive.actionBuilder(secondSplinePose)
                .strafeToConstantHeading(firstCornerZone.position);
        Action a_innerSampleToCorner = innerSampleToCorner.build();

        TrajectoryActionBuilder toSecondSample = drive.actionBuilder(firstCornerZone)
                .strafeToConstantHeading(cornerToSecondSample.position)
                .splineToConstantHeading(secondSampleSpline.position, Math.toRadians(0))
                .strafeToConstantHeading(secondCornerZone.position);
        Action a_toSecondSample = toSecondSample.build();

        TrajectoryActionBuilder toThirdSample = drive.actionBuilder(secondCornerZone)
                .strafeToConstantHeading(cornerToThirdSample.position)
                .splineToConstantHeading(thirdSampleSpline.position, Math.toRadians(0))
                .strafeToConstantHeading(thirdCornerZone.position);
//                .strafeToConstantHeading(cornerZoneSecondPose.position);
        Action a_toThirdSample = toThirdSample.build();
//
//        TrajectoryActionBuilder cornerToCenterSample = drive.actionBuilder(cornerZone)
//                .strafeToLinearHeading(centerSamplePose.position, centerSamplePose.heading);
//        Action a_cornerToCenterSample = cornerToCenterSample.build();
//
//        TrajectoryActionBuilder centerSampleToCorner = drive.actionBuilder(centerSamplePose)
//                .strafeToLinearHeading(cornerZone.position, cornerZone.heading);
//        Action a_centerSampleToCorner = centerSampleToCorner.build();
//
//        TrajectoryActionBuilder cornerToOuterSample = drive.actionBuilder(cornerZone)
//                .strafeToLinearHeading(outerSamplePose.position, outerSamplePose.heading);
//        Action a_cornerToOuterSample = cornerToOuterSample.build();
//
//        TrajectoryActionBuilder outerSampleToSecondOuterSample = drive.actionBuilder(outerSamplePose)
//                .strafeToLinearHeading(outerSampleSecondPose.position, outerSampleSecondPose.heading);
//        Action a_outerSampleToSecondOuterSample = outerSampleToSecondOuterSample.build();
//
//        TrajectoryActionBuilder secondOuterSampleToOuterSample = drive.actionBuilder(outerSampleSecondPose)
//                .strafeToLinearHeading(outerSamplePose.position, outerSamplePose.heading);
//        Action a_secondOuterSampleToOuterSample = secondOuterSampleToOuterSample.build();
//
//        TrajectoryActionBuilder outerSampleToCorner = drive.actionBuilder(outerSamplePose)
//                .strafeToLinearHeading(cornerZone.position, cornerZone.heading);
//        Action a_outerSampleToCorner = outerSampleToCorner.build();



//        TrajectoryActionBuilder moveOutOfCorner = drive.actionBuilder(cornerZone)
//                .strafeToLinearHeading(cornerZoneSecondPose.position, cornerZoneSecondPose.heading);
//        Action a_moveOutOfCorner = moveOutOfCorner.build();
//
//        TrajectoryActionBuilder park = drive.actionBuilder(cornerZoneSecondPose)
//                .strafeToLinearHeading(cornerZone.position, cornerZone.heading);
//        Action a_park = park.build();

        waitForStart();

        Actions.runBlocking(
            new SequentialAction(
                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                new ServoAction(swing, RobotConstants.SWING_UP), // change this
                new SleepAction(1),
                new ParallelAction(
                    a_goToCenter,
                    slideToPosition(RobotConstants.SLIDE_HIGH_BAR_POS)
                ),
                new SequentialAction( // drop off sample
                    new SleepAction(0.35),
                    new ServoAction(claw, RobotConstants.CLAW_OPEN),
                    new SleepAction(0.35)
                ),
                new ParallelAction(
                        a_outFromCenter,
                        slideToPosition(RobotConstants.SLIDE_REST_POS)
                ),
                a_barToInnerSample,
                a_innerSampleToCorner,
                a_toSecondSample
//                a_toThirdSample
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
                    slideMotor.getCurrentPosition()<(distance+distanceTolerance) || this.time.seconds()>3.5)) {
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


