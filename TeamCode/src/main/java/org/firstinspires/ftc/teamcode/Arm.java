package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//TODO: ExtendOut class and extendOut method could maybe be combined into one method. tldr; messy code :3
public class Arm {
    private DcMotor baseArmMotor;
    private DcMotor extendArmMotor;

    PIDController basePID;
    PIDController armPID;

    int baseArm_TargetPosition = 0;
    int extendArm_TargetPosition = 0;

    public Arm(HardwareMap map) {
        baseArmMotor = map.get(DcMotor.class, "baseArmMotor");
        extendArmMotor = map.get(DcMotor.class, "extendArmMotor");

        baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        baseArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        basePID = new PIDController(0,0,0); //TODO: tune PID coefficients
        armPID = new PIDController(0,0,0); //TODO: tune PID coefficients
    }

//    public void extendOut() {
//        if (basePID.getLastError() > 100 || armPID.getLastError() > 100) {
//            baseArmMotor.setPower(
//                    basePID.update(baseArm_TargetPosition, baseArmMotor.getCurrentPosition())
//            );
//
//            extendArmMotor.setPower(
//                    armPID.update(extendArm_TargetPosition, extendArmMotor.getCurrentPosition())
//            );
//
//    }

//    public class ExtendOut implements Action {
//
//        // This is has to return a boolean. The "Action" ends once it returns false.
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            baseArmMotor.setPower(
//                    basePID.update(baseArm_TargetPosition, baseArmMotor.getCurrentPosition())
//            );
//
//            extendArmMotor.setPower(
//                    armPID.update(extendArm_TargetPosition, extendArmMotor.getCurrentPosition())
//            );
//
//
//            //If the last errors of both the PIDs are under 100 ticks, return false and end the action.
//            if(basePID.getLastError() < 123123 && armPID.getLastError() < 123123) {
//                return false;
//            }
//            return true;
//
//        }
//
//    }
//    public Action extendOut() {
//        return new ExtendOut();
//    }

    public Action extendOut() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return packet -> {
            baseArmMotor.setPower(
                basePID.calculate(baseArm_TargetPosition, baseArmMotor.getCurrentPosition())
            );

            extendArmMotor.setPower(
                armPID.calculate(extendArm_TargetPosition, extendArmMotor.getCurrentPosition())
            );

            //TODO: 100 is a default tick count. Adjust to real motor constraints.
            //If the last errors of both the PIDs are under 100 ticks, return false and end the action.
            if(basePID.getLastError() < 100 && armPID.getLastError() < 100) {
                return false;
            }
            return true;
        };
    }
}
