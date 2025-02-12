package frc.robot.util.math;

import static java.lang.Math.abs;

import frc.robot.Setup;

public class Deadbands{
    public static Deadbands instance = new Deadbands();

    public static Deadbands getInstance() {
        if (instance == null) {
            instance = new Deadbands();
        }
        return instance;
    }   
    private Deadbands(){}

    public boolean isWithin(double valueA, double valueB, double deadband){
        return (valueA < valueB+deadband)&&(valueA>valueB-deadband);
    }

    public boolean isOutside(double valueA, double valueB, double deadband){
        return (valueA > valueB+deadband)&&(valueA<valueB-deadband);
    }
    public boolean isGreater(double valueA, double deadband){
        return valueA>deadband || valueA<-deadband;
    }
}