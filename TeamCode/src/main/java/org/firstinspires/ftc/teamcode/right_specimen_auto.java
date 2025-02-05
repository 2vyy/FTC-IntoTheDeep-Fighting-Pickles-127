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
        Vector2d innerSplinePose = new Vector2d(35, -20);
        Vector2d innerSplineSecondPose = new Vector2d(45.5, -15);
        Vector2d centerSplinePose = new Vector2d(56, -20);
        Vector2d outerSplinePose = new Vector2d(62, -20);


        Pose2d innerSamplePose = new Pose2d(45.5, -12, Math.toRadians(90));
        Pose2d centerSamplePose = new Pose2d(56, -12, Math.toRadians(90));

        Pose2d innerSamplePushPose = new Pose2d(45.5, -55, Math.toRadians(90));
        Pose2d centerSamplePushPose = new Pose2d(56, -55, Math.toRadians(90));
        Pose2d outerSamplePushPose = new Pose2d(62, -55, Math.toRadians(90));

        Pose2d waitPose = new Pose2d(55, -40, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(55, -61, Math.toRadians(90));

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder goToCenter = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(centerBarPose.position);
        Action a_goToCenter = goToCenter.build();

        TrajectoryActionBuilder outFromCenter = drive.actionBuilder(centerBarPose)
                .strafeToConstantHeading(centerBarSecondPose.position);
        Action a_outFromCenter = outFromCenter.build();

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
                            a_outFromCenter,
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