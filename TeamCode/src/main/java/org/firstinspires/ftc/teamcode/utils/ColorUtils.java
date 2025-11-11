package org.firstinspires.ftc.teamcode.utils;

public class ColorUtils {

    public static boolean isBlack(int red, int green, int blue) {
        return red < 50 && green < 50 && blue < 50;
    }

    public static boolean isWhite(int red, int green, int blue) {
        return red > 200 && green > 200 && blue > 200;
    }

    public static boolean isGreen(int red, int green, int blue) {
        return green > red && green > blue && green > 100;
    }

    public static boolean isPurple(int red, int green, int blue) {
        return red > 100 && blue > 100 && green < 100;
    }
}
