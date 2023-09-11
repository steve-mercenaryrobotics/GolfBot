package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class RobotConstants {
    //Ball seek rotation power factor
    public static double ROTATE_FACTOR = 1.0 / 400;

    //Blue values
    public static int color_lower_h = 50;
    public static int color_lower_s = 100;
    public static int color_lower_v = 100;
    public static int color_upper_h = 185;
    public static int color_upper_s = 255;
    public static int color_upper_v = 255;
    public static int elementType = 0;
    public static int kernelSize = 1;
    public static int drawContours = 1;
    public static int drawRectangle = 1;
    public static int drawCircle = 1;
    public static int minBallArea = 300;

    public static double ballX = 0;
    public static double ballY = 0;
    public static boolean ballExists = false;
}