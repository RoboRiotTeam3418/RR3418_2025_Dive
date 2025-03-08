package frc.robot.util.math;

public class DeadbandUtils {

    public static boolean isWithin(double valueA, double valueB, double deadband) {
        return (valueA < valueB + deadband) && (valueA > valueB - deadband);
    }

    public static boolean isOutside(double valueA, double valueB, double deadband) {
        return (valueA > valueB + deadband) || (valueA < valueB - deadband);
    }

    public static boolean isGreater(double valueA, double deadband) {
        return Math.abs(valueA) > deadband;
    }

    public static boolean isLess(double valueA, double deadband) {
        return Math.abs(valueA) < deadband;
    }
}