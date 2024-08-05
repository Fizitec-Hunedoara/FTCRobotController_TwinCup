package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;
@Config
public class Parametri{
    public enum DetectionTypes {
        DAY_BLUE,
        NIGHT_BLUE,
        DAY_RED,
        NIGHT_RED
    }

    public static double Day_Hhigh_Blue = 180, Day_Shigh_Blue = 255, Day_Vhigh_Blue = 255, Day_Hlow_Blue = 0, Day_Slow_Blue = 150, Day_Vlow_Blue = 150;
    public static double Night_Hhigh_Blue = 180, Night_Shigh_Blue = 255, Night_Vhigh_Blue = 255, Night_Hlow_Blue = 90, Night_Slow_Blue = 150, Night_Vlow_Blue = 100;
    public static double Day_Hhigh_Red = 180, Day_Shigh_Red = 255, Day_Vhigh_Red = 255, Day_Hlow_Red = 0, Day_Slow_Red = 150, Day_Vlow_Red = 150;
    public static double Night_Hhigh_Red = 180, Night_Shigh_Red = 255, Night_Vhigh_Red = 255, Night_Hlow_Red = 0, Night_Slow_Red = 150, Night_Vlow_Red = 150;
    public static int CV_kernel_pult_size = 5, Webcam_w = 640, Webcam_h = 480, CV_rect_x1 = 0, CV_rect_y1 = 0, CV_rect_x2 = 640, CV_rect_y2 = 480;
    public static DetectionTypes CV_detectionType = DetectionTypes.DAY_BLUE;

    public static double pslider = 0.01, islider = 0.0001, dslider = 0.015;
}