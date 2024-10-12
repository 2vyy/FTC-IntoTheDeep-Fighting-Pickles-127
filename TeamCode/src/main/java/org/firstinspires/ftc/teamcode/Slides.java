package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private DcMotor slideMotor;
    private Servo basketFlipper;

    private PIDFController slidePID;

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

    public Action slideUp() {
        //has to return boolean
        return packet -> {

            //optimal power
            slideMotor.setPower(
                    slidePID.calculate(
                            RobotConstants.SLIDE_EXTEND_POS,
                            slideMotor.getCurrentPosition()
                    )
            );

            if(slidePID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE) {
                return false;
            }
            return true;
        };
    }

    public Action slideDown() {
        //has to return boolean
        return packet -> {
            slideMotor.setPower(
                    slidePID.calculate(
                            RobotConstants.SLIDE_REST_POS,
                            slideMotor.getCurrentPosition()
                    )
            );

            if(slidePID.getLastError() < RobotConstants.PID_ERROR_TOLERANCE) {
                return false;
            }

            return true;
        };
    }

    public Action flipOut() {
        return packet -> {
            basketFlipper.setPosition(RobotConstants.BASKET_FLIPPER_OUT);
            return false;
        };
    }

    public Action flipIn() {
        return packet -> {
            basketFlipper.setPosition(RobotConstants.BASKET_FLIPPER_IN);
            return false;
        };
    }
}
