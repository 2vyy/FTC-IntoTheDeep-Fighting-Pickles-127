package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

// Magic numbers are bad. Global, static constants are good. Also, these can be live-tuned in FTC-Dashboard because of @Config
// See https://acmerobotics.github.io/ftc-dashboard/features.html
@Config
public class RobotConstants {
    public static int PID_ERROR_TOLERANCE = 35;
    // slide pid
    public static double SLIDE_kP = 0.004;
    public static double SLIDE_kI = 0.000001;
    public static double SLIDE_kD = 0.00001;
    public static double SLIDE_kF = 0.04;

    public static double SLIDE_SPEED = 0.7;

    public static int SLIDE_HIGH_BASKET_POS = 3650;
    public static int SLIDE_HIGH_BAR_POS = 1230;
    public static int SLIDE_REST_POS = 5;

    // swing servo
    public static double SWING_UP = 0.3;
    public static double SWING_SPECIMEN = 0.41;
    public static double SWING_DOWN = 0.55;
    public static double SWING_VERY_DOWN = 0.65;
    public static double SWING_AUTO_SAMPLE = 0.56;
    public static double SWING_AUTO_PARK = 0.2;
    // claw
    public static double CLAW_OPEN = 0.4;
    public static double CLAW_CLOSE = 0.2;
}