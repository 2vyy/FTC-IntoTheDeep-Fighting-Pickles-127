package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

// Magic numbers are bad. Global, static constants are good. Also, these can be live-tuned in FTC-Dashboard because of @Config
// See https://acmerobotics.github.io/ftc-dashboard/features.html
@Config
public class RobotConstants {

    public static int PID_ERROR_TOLERANCE = 10;
    // slide pid
    public static double SLIDE_kP = 0.004;//TODO: tune PID coefficients

    public static double SLIDE_kI = 0;
    public static double SLIDE_kD = 0;
    public static double SLIDE_kF = 0.02;


    public static int SLIDE_HIGH_BASKET_POS = 2100;
    public static int SLIDE_HIGH_BAR_POS = 2650;
    public static int SLIDE_HIGH_BAR_POS_SECOND = 2550;
    public static int SLIDE_REST_POS = 300;

    // swing servo
    public static double SWING_UP = 0.3;//TODO: tune servo positions
    public static double SWING_SPECIMEN = 0.45;
    public static double SWING_DOWN = 0.65;

    // claw
    public static double CLAW_OPEN = 0.1;
    public static double CLAW_CLOSE = 0;
}

