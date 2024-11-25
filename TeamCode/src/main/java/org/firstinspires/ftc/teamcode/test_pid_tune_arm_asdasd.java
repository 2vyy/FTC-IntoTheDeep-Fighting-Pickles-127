package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="pid_tuning_teleop")
public class test_pid_tune_arm_asdasd extends LinearOpMode {
    private DcMotor baseArmMotor;
    private DcMotor extendArmMotor;

    private CRServo roller;

    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    PIDFController extendPID;

    boolean extend = true;

    double a = 0.0;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        baseArmMotor = hardwareMap.get(DcMotor.class, "baseArmMotor");
        extendArmMotor = hardwareMap.get(DcMotor.class, "extendArmMotor");
        roller = hardwareMap.get(CRServo.class, "roller");

        baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        baseArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        baseArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        baseArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //ftc dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        extendPID = new PIDFController(
                RobotConstants.EXTEND_ARM_kP,
                RobotConstants.EXTEND_ARM_kI,
                RobotConstants.EXTEND_ARM_kD,
                RobotConstants.EXTEND_ARM_kF
        );



        waitForStart();

        while(opModeIsActive()) {

//            if(gamepad1.a) {
//                roller.setPower(1);
//            } else if (gamepad1.b) {
//                roller.setPower(-1);
//            } else {
////                roller.setPower(0);
//            }



            if(gamepad1.a) {
                extend = true;
            } else if (gamepad1.b) {
                extend = false;
            }

            telemetry.addData("extendCurrentPosition", extendArmMotor.getCurrentPosition());
            telemetry.addData("baseCurrentPosition", baseArmMotor.getCurrentPosition());
            telemetry.addData("asd", 0.0);
            telemetry.update();

            armAction();
        }
    }

    public void armAction() {
        if(extend) {
//                    extendArmMotor.setPower(calculate(RobotConstants.EXTEND_ARM_EXTEND_POS,
//                    extendArmMotor.getCurrentPosition(), RobotConstants.EXTEND_ARM_kP,
//                    RobotConstants.EXTEND_ARM_kI, RobotConstants.EXTEND_ARM_kD,
//                    RobotConstants.EXTEND_ARM_kF));

            extendArmMotor.setPower(
                    extendPID.calculate(
                            RobotConstants.EXTEND_ARM_EXTEND_POS,
                            extendArmMotor.getCurrentPosition()
                    )
            );

            baseArmMotor.setPower(calculate(RobotConstants.BASE_ARM_EXTEND_POS,
                    baseArmMotor.getCurrentPosition(), RobotConstants.BASE_ARM_kP,
                    RobotConstants.BASE_ARM_kI, RobotConstants.BASE_ARM_kD,
                    RobotConstants.BASE_ARM_kF));

            telemetry.addData("extendTargetPosition", RobotConstants.EXTEND_ARM_EXTEND_POS);
            telemetry.addData("baseTargetPosition", RobotConstants.BASE_ARM_EXTEND_POS);

        } else {

//            extendArmMotor.setPower(calculate(RobotConstants.EXTEND_ARM_REST_POS,
//                    extendArmMotor.getCurrentPosition(), RobotConstants.EXTEND_ARM_kP,
//                    RobotConstants.EXTEND_ARM_kI, RobotConstants.EXTEND_ARM_kD,
//                    RobotConstants.EXTEND_ARM_kF));

            extendArmMotor.setPower(
                    extendPID.calculate(
                            RobotConstants.EXTEND_ARM_REST_POS,
                            extendArmMotor.getCurrentPosition()
                    )
            );

            baseArmMotor.setPower(calculate(RobotConstants.BASE_ARM_REST_POS,
                    baseArmMotor.getCurrentPosition(), RobotConstants.BASE_ARM_kP,
                    RobotConstants.BASE_ARM_kI, RobotConstants.BASE_ARM_kD,
                    RobotConstants.BASE_ARM_kF));

            telemetry.addData("extendTargetPosition", RobotConstants.EXTEND_ARM_REST_POS);
            telemetry.addData("baseTargetPosition", RobotConstants.BASE_ARM_REST_POS);

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
