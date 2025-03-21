package frc.robot.util.drivers;

public class Toggles{
    static boolean backToggle=true;

    public static boolean getBackwards() {
        return backToggle;
    }
    public static void toggleBackwards() {
        backToggle=!backToggle;
    }
}