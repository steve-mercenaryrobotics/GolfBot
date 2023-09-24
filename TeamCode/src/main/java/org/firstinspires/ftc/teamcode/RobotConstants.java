package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class RobotConstants {
    //Ball seek rotation power factor
    public static double ROTATE_FACTOR_LARGE = 1.0 / 500;
    public static double ROTATE_FACTOR_MEDIUM = 1.0 / 700;
    public static double ROTATE_FACTOR_SMALL = 1.0 / 900;

    //HSV calculated from here...
    //https://colorizer.org/
    //NOTE... Ranges need scaled as follows...
    //H = 0-360 converted to 0-180 for OpenCV
    //S&V = 0-100 converted to 0-255 for OpenCV
    //Blue values
    /*
    public static int color_lower_h = 50;
    public static int color_lower_s = 100;
    public static int color_lower_v = 100;
    public static int color_upper_h = 185;
    public static int color_upper_s = 255;
    public static int color_upper_v = 255;
    //Orange
    public static int color_lower_h = 8;
    public static int color_upper_h = 25;
    public static int color_lower_s = 200;
    public static int color_upper_s = 255;
    public static int color_lower_v = 90;
    public static int color_upper_v = 255;
    */
    //Yellow values
    public static int color_lower_h = 23;
    public static int color_upper_h = 41;
    public static int color_lower_s = 94;
    public static int color_upper_s = 255;
    public static int color_lower_v = 163;
    public static int color_upper_v = 255;

    public static int elementType = 0;
    public static int kernelSize = 1;
    public static int drawContours = 0;
    public static int drawRectangle = 0;
    public static int drawCircle = 1;
    public static int minBallArea = 50;

    public static double ballX = 0.0;
    public static double ballY = 0.0;
    public static double foundBallArea = 0.0;
    public static boolean ballExists = false;
    public static double readyToGrabY = 180.0;
    public static double captureSpeed = -0.1;
    public static double trackFactor = .001;
    public static int delayTimer = 20;
    public static int captureDistance = 370;

    public static int clubForward = -18;
    public static int clubBack = 120;
}