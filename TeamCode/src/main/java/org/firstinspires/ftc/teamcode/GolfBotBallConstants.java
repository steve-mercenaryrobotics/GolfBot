package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class GolfBotBallConstants {

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
    /*public static int color_lower_h = 20;
    public static int color_upper_h = 35;
    public static int color_lower_s = 60;
    public static int color_upper_s = 255;
    public static int color_lower_v = 130;
    public static int color_upper_v = 255;*/

    // Kevin's values
    public static int color_lower_h = 20;
    public static int color_upper_h = 35;
    public static int color_lower_s = 70;
    public static int color_upper_s = 255;
    public static int color_lower_v = 130;
    public static int color_upper_v = 255;

    public static int elementType = 0;
    public static int kernelSize = 2;
    public static int drawContours = 0;
    public static int drawRectangle = 0;
    public static int drawCircle = 1;
    public static int minBallArea = 40;
}
