package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="pid_tuning_teleop")
public class test_pid_tune_arm_asdasd extends LinearOpMode {
    private DcMotor slideMotor;
    private Servo swing;
    private Servo claw;

    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    boolean extend = false;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        swing = hardwareMap.get(Servo.class, "swing");
        claw = hardwareMap.get(Servo.class, "claw");

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //ftc dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                extend = true;
            } else if (gamepad1.b) {
                extend = false;
            }

            telemetry.addData("slideCurrentPosition", slideMotor.getCurrentPosition());
//            telemetry.addData("asd", 0.0);
            telemetry.update();

            armAction();
        }
    }

    public void armAction() {
        if(extend) {
            slideMotor.setPower(calculate(RobotConstants.SLIDE_REST_POS,
                    slideMotor.getCurrentPosition(), RobotConstants.SLIDE_kP,
                    RobotConstants.SLIDE_kI, RobotConstants.SLIDE_kD,
                    RobotConstants.SLIDE_kF));

        } else {
            slideMotor.setPower(calculate(RobotConstants.SLIDE_HIGH_BASKET_POS,
                    slideMotor.getCurrentPosition(), RobotConstants.SLIDE_kP,
                    RobotConstants.SLIDE_kI, RobotConstants.SLIDE_kD,
                    RobotConstants.SLIDE_kF));

        }
    }

    public double calculate(double target, double state, double Kp, double Ki, double Kd, double Kf) {
        // calculate the statistical error
        double error = target - state;

        // rate of change of the error, simple displacement over time
        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + Kf;

        lastError = error;

        // reset the timer for next time
        timer.reset();
        return out;
    }
}
