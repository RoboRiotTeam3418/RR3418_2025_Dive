package frc.robot.util.drivers;

public class Toggles{
    static boolean secondaryToggle=true;

    public static boolean getSecondaryToggle() {
        return secondaryToggle;
    }
    public static void toggleSecondary() {
        secondaryToggle=!secondaryToggle;
    }
}