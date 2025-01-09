package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder startToCenter = drive.actionBuilder(initialPose)
                .stopAndAdd(new ServoAction(claw, RobotConstants.CLAW_CLOSE))
                .waitSeconds(.25)
                .stopAndAdd(new ServoAction(swing, RobotConstants.SWING_UP));

//        TrajectoryActionBuilder startToCenter = drive.actionBuilder(initialPose)
//                .splineToConstantHeading(new Vector2d(24, 24), Math.PI / 4);

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
                            this.currPose
                    )
            );

            //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough so return false and end the action.
            if(Math.abs(this.currPose-this.position)<RobotConstants.PID_ERROR_TOLERANCE) {
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


