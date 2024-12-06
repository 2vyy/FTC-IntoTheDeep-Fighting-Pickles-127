package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.acmerobotics.roadrunner.InstantAction;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    private DcMotor slideMotor;
    private Servo swing;
    private Servo claw;

    private PIDFController slidePID;

    private enum SlideState {
        REST,
        HIGH_BASKET,
        HIGH_BAR
    }

    SlideState slideState = SlideState.REST;

    public Slide(HardwareMap map) {
        slideMotor = map.get(DcMotor.class, "slideMotor");
        swing = map.get(Servo.class, "swing");
        claw = map.get(Servo.class, "claw");

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
    public Action toHighBar() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slideMotor.setPower(
                        slidePID.calculate(
                                RobotConstants.SLIDE_HIGH_BAR_POS,
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
                    swing.setPosition(RobotConstants.SWING_DOWN);
                    return false;
                }
                //if not, return true. Roadrunner will then repeat the action
                return true;
            }
        };
    }

    public Action slideToHighBar() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slideMotor.setPower(
                        slidePID.calculate(
                                RobotConstants.SLIDE_HIGH_BAR_POS,
                                slideMotor.getCurrentPosition()
                        )
                );

                //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough so return false and end the action.
                if(inTolerance()) {
                    swing.setPosition(RobotConstants.SWING_UP);
                    return false;
                }
                //if not, return true. Roadrunner will then repeat the action
                return true;
            }
        };
    }

    public Action slideToHighBasket() {
        // This is has to return a boolean. The "Action" ends once it returns false.
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slideMotor.setPower(
                        slidePID.calculate(
                                RobotConstants.SLIDE_HIGH_BASKET_POS,
                                slideMotor.getCurrentPosition()
                        )
                );

                //If the last errors of both the PIDs are under PID_ERROR_TOLERANCE ticks, the motors are close enough so return false and end the action.
                if(inTolerance()) {
                    swing.setPosition(RobotConstants.SWING_UP);
                    return false;
                }
                //if not, return true. Roadrunner will then repeat the action
                return true;
            }
        };
    }

//    public void slideAction(Gamepad gamepad, List<Action> teleActions) {
//        switch(slideState) {
//            case SLIDE_DOWN:
//                if(gamepad.dpad_up) {
//                    slideState = SlideState.SLIDE_UP;
//                    teleActions.add(slideUp());
//                }
//                break;
//            case SLIDE_UP:
//                if(inTolerance()) {
//                    flipOut();
//                    if(gamepad.dpad_down) {
//                        slideState = SlideState.SLIDE_DOWN;
//                        flipIn();
//                        teleActions.add(slideDown());
//                    }
//                }
//                break;
//        }
//    }


    //public void swingDown() {swing.setPosition(RobotConstants.SWING_DOWN);}
    public void swingUp() {swing.setPosition(RobotConstants.SWING_UP);}




    public void openClaw() {claw.setPosition(RobotConstants.CLAW_OPEN);}
    public void closeClaw() {claw.setPosition(RobotConstants.CLAW_CLOSE);}

    private boolean inTolerance() {
        return slidePID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE;
    }
}
