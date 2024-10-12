package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

// Magic numbers are bad. Global, static constants are good. Also, these can be live-tuned in FTC-Dashboard because of @Config
// See https://acmerobotics.github.io/ftc-dashboard/features.html
@Config
public class RobotConstants {
    public static int PID_ERROR_TOLERANCE = 0;
    // baseArmPID
    public static double BASE_ARM_kP = 0;//TODO: tune PID coefficients
    public static double BASE_ARM_kI = 0;
    public static double BASE_ARM_kD = 0;
    public static double BASE_ARM_kF = 0;
    public static int BASE_ARM_EXTEND_POS = 0;
    public static int BASE_ARM_REST_POS = 0;
    // extendArmPID
    public static double EXTEND_ARM_kP = 0;//TODO: tune PID coefficients
    public static double EXTEND_ARM_kI = 0;
    public static double EXTEND_ARM_kD = 0;
    public static double EXTEND_ARM_kF = 0;
    public static int EXTEND_ARM_EXTEND_POS = 0;
    public static int EXTEND_ARM_REST_POS = 0;
    // slidePID
    public static double SLIDE_kP = 0;//TODO: tune PID coefficients
    public static double SLIDE_kI = 0;
    public static double SLIDE_kD = 0;
    public static double SLIDE_kF = 0;

    public static int SLIDE_EXTEND_POS = 0;
    public static int SLIDE_REST_POS = 0;
    // basketFlipper
    public static double BASKET_FLIPPER_UP = 0;//TODO: tune servo positions
    public static double BASKET_FLIPPER_DOWN = 0;
}
