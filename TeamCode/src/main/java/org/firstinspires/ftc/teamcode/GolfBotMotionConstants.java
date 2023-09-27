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

    // Club Motion
    public static int maxClubCurrent = 2800;
    public static int clubMotorTolerance = 2;
    public static int clubForward = -12;
    public static int clubBack = 120;
    public static double hitBallPower = 0.3;
    public static int swingDelayTimer = 10;

    // Drive Past Ball and turn settings
    public static int drivePastDistance = 1000;
    public static double drivePastSpeed = 0.3;
    public static double rotatePower = 0.2;
    public static double rotatePrecision = 5.0;

    // TODO: Implement Ball Alignment
    // public static int captureLowerDist = 70;
    // public static int captureHigherDist = 90;

}
