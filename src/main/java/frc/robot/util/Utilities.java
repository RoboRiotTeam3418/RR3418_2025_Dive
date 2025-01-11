package frc.robot.util;

//if input is less than buffer then the input is returned as zero
public class Utilities {
	public static double deadband(double input) {
		return deadband(input, 0.05);
	}

	public static double deadband(double input, double buffer) {
		if (Math.abs(input) < buffer) return 0;
		return input;
	}
}
