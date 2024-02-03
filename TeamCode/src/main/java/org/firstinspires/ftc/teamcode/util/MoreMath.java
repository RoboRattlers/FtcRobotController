package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MoreMath {

    public static double modulo(double a, double b) {
        return  (a % b + b) % b;
    }

    public static double map(double num, double oldMin, double oldMax, double newMin, double newMax, boolean clamp) {
        if (oldMin == oldMax || newMin == newMax) {
            return newMin;
        }
        double newValue = (num - oldMin)/(oldMax - oldMin) * (newMax - newMin) + newMin;
        if (clamp) {
            newValue = clamp(newValue, newMin, newMax);
        }
        return newValue;
    }

    public static double clamp(double num, double min, double max) {
        double max2 = Math.max(min, max);
        double min2 = Math.min(min, max);
        return Math.min(Math.max(num, min2), max2);
    }

    public static double clampMagnitude(double num, double mag) {
        return clamp(num, -Math.abs(mag), Math.abs(mag));
    }

    public static double normalizeAngle(double angle, AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.DEGREES) {
            return modulo((angle + 180), 360) - 180;
        } else {
            return modulo((angle + Math.PI), 2 * Math.PI) - Math.PI;
        }
    }

    public static double lerp(double a, double b, double alpha) {
        return a + (b - a) * alpha;
    }

    public static double round(double num, double interval) {
        return (Math.round(num/interval) * interval);
    }

}
