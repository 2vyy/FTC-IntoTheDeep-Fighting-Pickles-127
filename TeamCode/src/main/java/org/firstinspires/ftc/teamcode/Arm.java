package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public enum ArmState {
        ARM_START,
        ARM_EXTEND,
        ARM_TRANSFER,
    }

    ArmState armState = ArmState.ARM_START;

    private DcMotor baseArmMotor;
    private DcMotor extendArmMotor;

    private CRServo roller;

    private PIDFController basePID;
    private PIDFController armPID;

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
        basePID = new PIDFController(
                RobotConstants.BASE_ARM_kP,
                RobotConstants.BASE_ARM_kI,
                RobotConstants.BASE_ARM_kD,
                RobotConstants.BASE_ARM_kF
        );
        armPID = new PIDFController(
                RobotConstants.EXTEND_ARM_kP,
                RobotConstants.EXTEND_ARM_kI,
                RobotConstants.EXTEND_ARM_kD,
                RobotConstants.EXTEND_ARM_kF
        );
    }

    //Roadrunner Action - This will constantly run *until* it returns false
    //Extend both motors out to their EXTEND_POS
    public Action A_extendOut() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> { // this weird -> symbol is called a lamdba if you're curious
            return extendOut();
        };
    }

    public boolean extendOut() {
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
    }

    public Action A_retractBack() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> {
            return retractBack();
        };
    }

    public boolean retractBack() {
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


    public Action rollInward() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> {
            roller.setPower(1);
            return false; //This is an instant action that doesn't need to wait on anything, so we can return false immediately
        };
    }

    public Action A_rollOutward() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> {
            rollOutward();
            return false;
        };
    }

    public void rollOutward() {
        roller.setPower(-1);
    }

    public Action stopRoller() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> {
            roller.setPower(0);
            return false;
        };
    }

    public void armAction(Gamepad gamepad) {
        switch(armState) {
            case ARM_START:
                if(gamepad.a) {
                    armState = ArmState.ARM_EXTEND;
                }
                break;
            case ARM_EXTEND:
                boolean extended = extendOut();
                if(!extended && gamepad.a) {
                    rollInward();
                }

                if(gamepad.b) {
                    armState = ArmState.ARM_TRANSFER;
                }
                break;
            case ARM_TRANSFER:
                boolean retracted = retractBack();
                if(!retracted) {
                    rollOutward();
                }
                if(gamepad.b) {
                    stopRoller();
                    armState = ArmState.ARM_START;
                }
                break;
            default:
                armState = ArmState.ARM_START;
        }
    }
}
