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

@Autonomous(name="🙏 right specimen auto 🙏")
public class right_specimen_auto extends LinearOpMode {

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

        Pose2d initialPose = new Pose2d(5, -61, Math.toRadians(90));
        Pose2d centerBarPose = new Pose2d(5, -33, Math.toRadians(90));
        Pose2d centerBarSecondPose = new Pose2d(30, -34, Math.toRadians(90));

        //Vector2d innerSplinePose = new Vector2d(35, -20);
        //Vector2d innerSplineSecondPose = new Vector2d(45.5, -15);
        //Vector2d centerSplinePose = new Vector2d(56, -20);
        //Vector2d outerSplinePose = new Vector2d(62, -20);

        Pose2d innerSplinePose = new Pose2d(35, -20, Math.toRadians(90));
        Pose2d innerSplineSecondPose = new Pose2d(45.5, -15, Math.toRadians(90));
        Pose2d centerSplinePose = new Pose2d(56, -20, Math.toRadians(90));
        Pose2d outerSplinePose = new Pose2d(62, -20, Math.toRadians(90));

        Pose2d innerSamplePose = new Pose2d(45.5, -12, Math.toRadians(90));
        Pose2d centerSamplePose = new Pose2d(56, -12, Math.toRadians(90));

        Pose2d innerSamplePushPose = new Pose2d(45.5, -55, Math.toRadians(90));
        Pose2d centerSamplePushPose = new Pose2d(56, -55, Math.toRadians(90));
        Pose2d outerSamplePushPose = new Pose2d(62, -55, Math.toRadians(90));

        Pose2d waitPose = new Pose2d(55, -40, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(55, -61, Math.toRadians(90));

        Pose2d backOutFromBarPose = new Pose2d(5, -42, Math.toRadians(90));
        Pose2d towardSpecimenGroupPose = new Pose2d(35, -42,Math.toRadians(90));

        //Pose2d goToFirstSpecimenPose = new Pose2d(-4, -34,Math.toRadians(90));
        //Pose2d goToSecondSpecimenPose = new Pose2d(4, -34,Math.toRadians(90));
        //Pose2d goToThirdSpecimenPose = new Pose2d(8, -34,Math.toRadians(90));

        Pose2d placeFirstSpecimenPose = new Pose2d(-4, -34,Math.toRadians(90));
        Pose2d placeSecondSpecimenPose = new Pose2d(4, -34,Math.toRadians(90));
        Pose2d placeThirdSpecimenPose = new Pose2d(8, -34,Math.toRadians(90));



        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder goToCenter = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(centerBarPose.position);
        Action a_goToCenter = goToCenter.build();

        TrajectoryActionBuilder outFromCenter = drive.actionBuilder(centerBarPose)
                .strafeToConstantHeading(centerBarSecondPose.position);
        Action a_outFromCenter = outFromCenter.build();

        TrajectoryActionBuilder backOutFromBar = drive.actionBuilder(centerBarSecondPose)
                .strafeTo(backOutFromBarPose.position);
        Action a_backOutFromBar = backOutFromBar.build();

        TrajectoryActionBuilder towardSpecimenGroup = drive.actionBuilder(backOutFromBarPose)
                .strafeToConstantHeading(new Vector2d(35,-42));
        Action a_towardSpecimenGroup = towardSpecimenGroup.build();

        TrajectoryActionBuilder goToFirstSpecimen = drive.actionBuilder(towardSpecimenGroupPose)
                .splineToConstantHeading(innerSplinePose.position, Math.toRadians(90))
                .splineToConstantHeading(innerSplineSecondPose.position, Math.toRadians(0));
        Action a_goToFirstSpecimen = goToFirstSpecimen.build();

        TrajectoryActionBuilder pushFirstSpecimen = drive.actionBuilder(innerSplineSecondPose)
                .strafeToLinearHeading(innerSamplePushPose.position, innerSamplePushPose.heading)
                .strafeToLinearHeading(innerSamplePose.position, innerSamplePose.heading);
        Action a_pushFirstSpecimen = pushFirstSpecimen.build();

        TrajectoryActionBuilder goToSecondSpecimen = drive.actionBuilder(innerSamplePose)
                .splineToConstantHeading(centerSplinePose.position, Math.toRadians(270));
        Action a_goToSecondSpecimen = goToSecondSpecimen.build();

        TrajectoryActionBuilder pushSecondSpecimen = drive.actionBuilder(centerSplinePose)
                .strafeToLinearHeading(centerSamplePushPose.position, centerSamplePushPose.heading)
                .strafeToLinearHeading(centerSamplePose.position, centerSamplePose.heading);
        Action a_pushSecondSpecimen = pushSecondSpecimen.build();

        TrajectoryActionBuilder goToThirdSpecimen = drive.actionBuilder(centerSamplePose)
                .splineToConstantHeading(outerSplinePose.position, Math.toRadians(270));
        Action a_goToThirdSpecimen = goToThirdSpecimen.build();

        TrajectoryActionBuilder pushThirdSpecimen = drive.actionBuilder(outerSplinePose)
                .strafeToLinearHeading(outerSamplePushPose.position, outerSamplePushPose.heading);
        Action a_pushThirdSpecimen = pushThirdSpecimen.build();

        TrajectoryActionBuilder readyToPickUpSpecimen = drive.actionBuilder(outerSamplePushPose)
                .strafeToLinearHeading(waitPose.position, waitPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading);
        Action a_readyToPickUpSpecimen = readyToPickUpSpecimen.build();

        TrajectoryActionBuilder readyToPickUpSecondSpecimen = drive.actionBuilder(placeFirstSpecimenPose)
                .strafeToLinearHeading(parkPose.position, parkPose.heading);
        Action a_readyToPickUpSecondSpecimen = readyToPickUpSecondSpecimen.build();

        TrajectoryActionBuilder readyToPickUpThirdSpecimen = drive.actionBuilder(placeSecondSpecimenPose)
                .strafeToLinearHeading(parkPose.position, parkPose.heading);
        Action a_readyToPickUpThirdSpecimen = readyToPickUpThirdSpecimen.build();

        TrajectoryActionBuilder finalPark = drive.actionBuilder(placeThirdSpecimenPose)
                .strafeToLinearHeading(parkPose.position, parkPose.heading);
        Action a_finalPark = finalPark.build();

        TrajectoryActionBuilder firstSpecimen = drive.actionBuilder(centerBarPose)
                //.strafeTo(new Vector2d(-4,-34));
                .strafeToConstantHeading(placeFirstSpecimenPose.position);
        Action a_firstSpecimen = firstSpecimen.build();

        TrajectoryActionBuilder secondSpecimen = drive.actionBuilder(centerBarPose)
                .strafeToConstantHeading(placeSecondSpecimenPose.position);
        Action a_secondSpecimen = secondSpecimen.build();

        TrajectoryActionBuilder thirdSpecimen = drive.actionBuilder(centerBarPose)
                .strafeToConstantHeading(placeThirdSpecimenPose.position);
        Action a_thirdSpecimen = thirdSpecimen.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                        new ServoAction(swing, RobotConstants.SWING_UP),
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
                            a_backOutFromBar,
                            slideToPosition(RobotConstants.SLIDE_REST_POS)
                        ),

                        new SequentialAction( // drop off sample
                            a_towardSpecimenGroup,
                                a_goToFirstSpecimen,
                                a_pushFirstSpecimen,
                                a_goToSecondSpecimen,
                                a_pushSecondSpecimen,
                                a_goToThirdSpecimen,
                                a_pushThirdSpecimen,
                                a_readyToPickUpSpecimen,
                                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                                new ServoAction(swing, RobotConstants.SWING_UP),
                                new SleepAction(1)
                        ),
                        new ParallelAction(
                                a_firstSpecimen,
                                slideToPosition(RobotConstants.SLIDE_HIGH_BAR_POS)
                        ),

                        new SequentialAction( // drop off sample
                                new SleepAction(0.35),
                                new ServoAction(claw, RobotConstants.CLAW_OPEN),
                                new SleepAction(0.35)
                        ),

                        new SequentialAction(
                                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                                new ServoAction(swing, RobotConstants.SWING_UP),
                                new SleepAction(1)
                        ),

                        new ParallelAction(
                                a_readyToPickUpSecondSpecimen,
                                slideToPosition(RobotConstants.SLIDE_REST_POS),
                                new SleepAction(.5)
                        ),

                        new SequentialAction(
                                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                                new ServoAction(swing, RobotConstants.SWING_UP),
                                new SleepAction(1)
                        ),

                        new ParallelAction(
                                a_secondSpecimen,
                                slideToPosition(RobotConstants.SLIDE_HIGH_BAR_POS),
                                new SleepAction(.5)
                        ),

                        new SequentialAction( // drop off sample
                                new SleepAction(0.35),
                                new ServoAction(claw, RobotConstants.CLAW_OPEN),
                                new SleepAction(0.35)
                        ),

                        new ParallelAction(
                                a_readyToPickUpThirdSpecimen,
                                slideToPosition(RobotConstants.SLIDE_REST_POS),
                                new SleepAction(.5)
                        ),

                        new SequentialAction(
                                new ServoAction(claw, RobotConstants.CLAW_CLOSE),
                                new ServoAction(swing, RobotConstants.SWING_UP),
                                new SleepAction(1)
                        ),

                        new ParallelAction(
                                a_thirdSpecimen,
                                slideToPosition(RobotConstants.SLIDE_HIGH_BAR_POS),
                                new SleepAction(.5)
                        ),

                        new SequentialAction( // drop off sample
                                new SleepAction(0.35),
                                new ServoAction(claw, RobotConstants.CLAW_OPEN),
                                new SleepAction(0.35)
                            ),

                        new ParallelAction(
                                a_finalPark,
                                slideToPosition(RobotConstants.SLIDE_REST_POS)
                        )

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