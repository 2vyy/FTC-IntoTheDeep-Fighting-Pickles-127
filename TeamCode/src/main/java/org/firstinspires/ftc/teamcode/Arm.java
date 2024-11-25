package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Arm {

    private DcMotor baseArmMotor;
    private DcMotor extendArmMotor;

    private CRServo roller;

    private PIDFController basePID;
    private PIDFController extendPID;

    private enum ArmState {
        ARM_START,
        ARM_EXTEND,
        ARM_TRANSFER,
    }

    ArmState armState = ArmState.ARM_START;

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
        extendPID = new PIDFController(
                RobotConstants.EXTEND_ARM_kP,
                RobotConstants.EXTEND_ARM_kI,
                RobotConstants.EXTEND_ARM_kD,
                RobotConstants.EXTEND_ARM_kF
        );
    }

    //Roadrunner Action - This will constantly run *until* it returns false
    //Extend both motors out to their EXTEND_POS
    public Action extendOut() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                baseArmMotor.setPower(
                        basePID.calculate(
                                RobotConstants.BASE_ARM_EXTEND_POS,
                                baseArmMotor.getCurrentPosition()
                        )
                );

                extendArmMotor.setPower(
                        extendPID.calculate(
                                RobotConstants.EXTEND_ARM_EXTEND_POS,
                                extendArmMotor.getCurrentPosition()
                        )
                );

                //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough -> return false and end the action.
                if (!inTolerance()) {
                    return false; //repeat
                }
                return true; //end action
            }
        };
    }

    //Roadrunner Action - This will constantly run *until* it returns false
    //Extend both motors out to their EXTEND_POS
    public Action retractBack() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                baseArmMotor.setPower(
                        basePID.calculate(
                                RobotConstants.BASE_ARM_REST_POS,
                                baseArmMotor.getCurrentPosition()
                        )
                );

                extendArmMotor.setPower(
                        extendPID.calculate(
                                RobotConstants.EXTEND_ARM_REST_POS,
                                extendArmMotor.getCurrentPosition()
                        )
                );

                //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough -> return false and end the action.
                if (!inTolerance()) {
                    return false; //repeat
                }
                return true; //end action
            }
        };
    }

    public void armAction(Gamepad gamepad, List<Action> teleActions) {
        switch(armState) {
            case ARM_START:
                if(gamepad.a) {
                    armState = ArmState.ARM_EXTEND;
                    teleActions.add(extendOut());
                }
                break;
            case ARM_EXTEND:
                if(inTolerance() && gamepad.a) {
                    rollInward();
                }

                if(gamepad.b) {
                    armState = ArmState.ARM_TRANSFER;
                    teleActions.add(retractBack());
                }
                break;
            case ARM_TRANSFER:
                if(inTolerance()) {
                    rollOutward();
                }
                if(gamepad.b) {
                    stopRolling();
                    armState = ArmState.ARM_START;
                }
                break;
            default:
                armState = Arm.ArmState.ARM_START;
        }
    }

    public void rollInward() {roller.setPower(-1);}
    public void rollOutward() {roller.setPower(1);}
    public void stopRolling() {roller.setPower(-0);}

    private boolean inTolerance() {
        return basePID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE && extendPID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE;
    }


}
//
