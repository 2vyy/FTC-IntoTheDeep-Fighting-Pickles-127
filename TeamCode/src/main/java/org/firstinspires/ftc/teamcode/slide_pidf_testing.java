package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Slide PIDF Testing")
public class slide_pidf_testing extends LinearOpMode {
    DcMotor slideMotor;
    Servo swing;

    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();
    int target = 0;

    boolean extend = false;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        swing = hardwareMap.get(Servo.class, "swing");
        swing.setDirection(Servo.Direction.FORWARD);
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");



        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFController slidePID = new PIDFController(
                RobotConstants.SLIDE_kP,
                RobotConstants.SLIDE_kI,
                RobotConstants.SLIDE_kD,
                RobotConstants.SLIDE_kF
        );



        //ftc dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        swing.setPosition(RobotConstants.SWING_UP);

        while(opModeIsActive()) {
            if(gamepad1.a) {
                extend = true;

                target = RobotConstants.SLIDE_HIGH_BASKET_POS;

            } else if (gamepad1.b) {
                extend = false;
                target = RobotConstants.SLIDE_REST_POS;
            }

            telemetry.addData("slideCurrentPosition", slideMotor.getCurrentPosition());
            telemetry.addData("slideTarget", RobotConstants.SLIDE_HIGH_BASKET_POS);
            telemetry.addData("asd", 0.0);
            telemetry.addLine(slideMotor.getCurrentPosition()+"");

            telemetry.update();

            armAction();
        }
    }

    public void armAction() {
        if(extend) {
            slideMotor.setPower(calculate(RobotConstants.SLIDE_HIGH_BASKET_POS,
                    slideMotor.getCurrentPosition(), RobotConstants.SLIDE_kP,
                    RobotConstants.SLIDE_kI, RobotConstants.SLIDE_kD,
                    RobotConstants.SLIDE_kF));

        } else {
            slideMotor.setPower(calculate(RobotConstants.SLIDE_REST_POS,
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