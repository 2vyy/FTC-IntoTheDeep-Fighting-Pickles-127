package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name="test_auto")
public class AutoTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Servo swing = hardwareMap.get(Servo.class, "swing");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        PIDFController slidePID = new PIDFController(
                RobotConstants.SLIDE_kP,
                RobotConstants.SLIDE_kI,
                RobotConstants.SLIDE_kD,
                RobotConstants.SLIDE_kF
        );
        swing.setDirection(Servo.Direction.FORWARD);

        Pose2d initialPose = new Pose2d(-12, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder startToCenter = drive.actionBuilder(initialPose)
//                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_DOWN))
//                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_CLOSE))
                .stopAndAdd(new MotorAction(
                        slideMotor,
                        slidePID,
                        RobotConstants.SLIDE_REST_POS))
                .strafeTo(new Vector2d(-12, -59))
                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_CLOSE))
                .waitSeconds(1)
                .stopAndAdd(new MotorAction(
                        slideMotor,
                        slidePID,
                        RobotConstants.SLIDE_HIGH_BAR_POS))
                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_SPECIMEN))
                .strafeTo(new Vector2d(-12, -34))
                .stopAndAdd(new MotorAction(
                        slideMotor,
                        slidePID,
                        RobotConstants.SLIDE_HIGH_BAR_POS_SECOND))
                .strafeTo(new Vector2d(-12, -44))
                .waitSeconds(2.5)
                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_OPEN));


//                .strafeTo(new Vector2d(-12, -56))
//                .waitSeconds(.5)
//                .strafeTo(new Vector2d(-12, -58))
//                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_CLOSE))
//                .waitSeconds(1)
//                .stopAndAdd(new MotorAction(
//                        slideMotor,
//                        slidePID,
//                        RobotConstants.SLIDE_HIGH_BAR_POS))
//                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_SPECIMEN))
//                .strafeTo(new Vector2d(-12, -45));

//                .waitSeconds(2)
//                .stopAndAdd(new MotorAction(
//                        slideMotor,
//                        slidePID,
//                        RobotConstants.SLIDE_HIGH_BAR_POS))
//                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_SPECIMEN))
//                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_OPEN))
//                .waitSeconds(2)
//                ;

        Action a_startToCenter = startToCenter.build();

        waitForStart();

        Actions.runBlocking(
                a_startToCenter
        );
    }

    public class MotorAction implements Action {
        DcMotor slideMotor;
        PIDFController slidePIDF;
        double currPose;
        double position;

        public MotorAction(DcMotor m, PIDFController pidf, int p) {
            this.slideMotor = m;
            this.currPose = m.getCurrentPosition();
            this.slidePIDF = pidf;
            this.position = p;
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
                            currPose
                    )
            );

            //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough so return false and end the action.
            if(Math.abs(slideMotor.getCurrentPosition()-this.position)<20) {
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
