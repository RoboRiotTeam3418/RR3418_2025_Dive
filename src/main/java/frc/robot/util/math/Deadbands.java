package frc.robot.util.math;

import static java.lang.Math.abs;

import frc.robot.Setup;

public class Deadbands{
    public static Deadbands instance = new Deadbands();
    private Deadbands(){}

    public static boolean isWithin(double valueA, double valueB, double deadband){
        return (valueA < valueB+deadband)&&(valueA>valueB-deadband);
    }

    public static boolean isOutside(double valueA, double valueB, double deadband){
        return (valueA > valueB+deadband)&&(valueA<valueB-deadband);
    }
    public static boolean isGreater(double valueA, double deadband){
        return Math.abs(valueA)>deadband;
    }

    public static boolean isLess(double valueA, double deadband){
        return Math.abs(valueA)<deadband ;
    }
}