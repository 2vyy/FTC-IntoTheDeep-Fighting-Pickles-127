package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Slides {
    private DcMotor slideMotor;
    private Servo basketFlipper;

    private PIDFController slidePID;

    private PIDFController basePID;
    private PIDFController armPID;

    private enum SlideState {
        SLIDE_DOWN,
        SLIDE_UP,
    }

    SlideState slideState = SlideState.SLIDE_DOWN;

    public Slides(HardwareMap map) {
        slideMotor = map.get(DcMotor.class, "slideMotor");
        basketFlipper = map.get(Servo.class, "basketFlipper");

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //PID controller
        slidePID = new PIDFController(
                RobotConstants.SLIDE_kP,
                RobotConstants.SLIDE_kI,
                RobotConstants.SLIDE_kD,
                RobotConstants.SLIDE_kF
        );
    }

    //Roadrunner Action - This will constantly run *until* it returns false
    //Extend both motors out to their EXTEND_POS
    public Action slideUp() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slideMotor.setPower(
                        slidePID.calculate(
                                RobotConstants.SLIDE_EXTEND_POS,
                                slideMotor.getCurrentPosition()
                        )
                );

                //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough so return false and end the action.
                if(inTolerance()) {
                    return false;
                }
                //if not, return true. Roadrunner will then repeat the action
                return true;
            }
        };
    }

    //Roadrunner Action - This will constantly run *until* it returns false
    //Extend both motors out to their EXTEND_POS
    public Action slideDown() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slideMotor.setPower(
                        slidePID.calculate(
                                RobotConstants.SLIDE_REST_POS,
                                slideMotor.getCurrentPosition()
                        )
                );

                //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough so return false and end the action.
                if(inTolerance()) {
                    return false;
                }
                //if not, return true. Roadrunner will then repeat the action
                return true;
            }
        };
    }

    public void slideAction(Gamepad gamepad, List<Action> teleActions) {
        switch(slideState) {
            case SLIDE_DOWN:
                if(gamepad.dpad_up) {
                    slideState = SlideState.SLIDE_UP;
                    teleActions.add(slideUp());
                }
                break;
            case SLIDE_UP:
                if(inTolerance()) {
                    flipOut();
                    if(gamepad.dpad_down) {
                        slideState = SlideState.SLIDE_DOWN;
                        flipIn();
                        teleActions.add(slideDown());
                    }
                }
                break;
        }
    }

    public void flipOut() {basketFlipper.setPosition(RobotConstants.BASKET_FLIPPER_OUT);}
    public void flipIn() {basketFlipper.setPosition(RobotConstants.BASKET_FLIPPER_IN);}

    private boolean inTolerance() {
        return slidePID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE;
    }
}
