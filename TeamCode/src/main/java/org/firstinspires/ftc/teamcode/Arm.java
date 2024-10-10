package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private DcMotor baseArmMotor;
    private DcMotor extendArmMotor;

    private CRServo roller;

    private PIDController basePID;
    private PIDController armPID;

    //Simple constructor for an Arm object.
    public Arm(HardwareMap map) {
        baseArmMotor = map.get(DcMotor.class, "baseArmMotor");
        extendArmMotor = map.get(DcMotor.class, "extendArmMotor");
        roller = map.get(CRServo.class, "roller");

        baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        baseArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //these motors probably need to be reversed but all good :3

        //We have 2 motors, so we need 2 PIDs
        basePID = new PIDController(
                RobotConstants.BASE_ARM_kP,
                RobotConstants.BASE_ARM_kI,
                RobotConstants.BASE_ARM_kD
        );
        armPID = new PIDController(
                RobotConstants.EXTEND_ARM_kP,
                RobotConstants.EXTEND_ARM_kI,
                RobotConstants.EXTEND_ARM_kD
        );
    }

    //Roadrunner Action - This will constantly run *until* it returns false
    //Extend both motors out to their EXTEND_POS
    public Action extendOut() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> { // this weird -> symbol is called a lamdba if you're curious

            // Ask the PID, "This is my goal and this is my current position. Give me an optimal power", and set the motor to that power
            baseArmMotor.setPower(
                basePID.calculate(
                        RobotConstants.BASE_ARM_EXTEND_POS,
                        baseArmMotor.getCurrentPosition()
                )
            );

            extendArmMotor.setPower(
                armPID.calculate(
                        RobotConstants.EXTEND_ARM_EXTEND_POS,
                        extendArmMotor.getCurrentPosition()
                )
            );

            //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough -> return false and end the action.
            if(basePID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE && armPID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE) {
                return false;
            }
            //if not, return true. Roadrunner will then repeat the action
            return true;
        };
    }

    public Action retractBack() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> {
            baseArmMotor.setPower(
                    basePID.calculate(
                            RobotConstants.BASE_ARM_REST_POS,
                            baseArmMotor.getCurrentPosition()
                    )
            );

            extendArmMotor.setPower(
                    armPID.calculate(
                            RobotConstants.EXTEND_ARM_REST_POS,
                            extendArmMotor.getCurrentPosition()
                    )
            );

            //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough -> return false and end the action.
            if(basePID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE && armPID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE) {
                return false;
            }
            return true;
        };
    }

    public Action rollInward() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> {
            roller.setPower(1);
            return false; //This is an instant action that doesn't need to wait on anything, so we can return false immediately
        };
    }

    public Action rollOutward() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> {
            roller.setPower(-1);
            return false;
        };
    }

    public Action stopRoller() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> {
            roller.setPower(0);
            return false;
        };
    }


}
