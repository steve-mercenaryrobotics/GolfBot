package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class GolfBotMotionConstants {
    //Ball seek rotation power factor
    public static double ROTATE_FACTOR_LARGE = 1.0 / 600;
    public static double ROTATE_FACTOR_MEDIUM = 1.0 / 700;
    public static double ROTATE_FACTOR_SMALL = 1.0 / 900;
    public static double readyToGrabY = 180.0;
    public static double captureSpeed = -0.1;
    public static double trackFactor = .001;
    public static int delayTimer = 20;
    public static int captureDistance = 370;
    public static int clubForward = -18;
    public static int clubBack = 120;

}
