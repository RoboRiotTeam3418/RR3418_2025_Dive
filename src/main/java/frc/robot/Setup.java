package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.util.drivers.Gyroscope;
import frc.robot.util.drivers.NavX;

public class Setup {
    
  public static Setup instance = new Setup();

  public static Setup getInstance() {
    if (instance == null) {
      instance = new Setup();
    }
	  return instance;
  }   


 //---------------------------------------------------------Swerve Drive------------------------------------------------------------------

  //measurments of robot in meters from center of wheel (19.25 inches squared, 39.37 inches in a meter)
  public double TRACKWIDTH = 0.607;
  public double WHEELBASE = 0.597;
    
  //offset of wheels sets the angle to start - CHANGE DIS BRO
  public double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(123.579545);
  public double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(260.795455);
  public double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(-4.303369);
  public double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(18.579545);

  //finds position of the wheels based on the position of the center
  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
    new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),        
    new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
    new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0)
  );

 //----------------------------------------------------------Primary----------------------------------------------------------------------

  //Flight Stick (Primary)
  private static Joystick primaryJoystick = new Joystick(0);

  public Joystick getPrimaryJoystick() {
    return primaryJoystick;
  }
  

  //movement
  public double getPrimaryX(){
    return primaryJoystick.getRawAxis(0);
  }
  
  public double getPrimaryY(){
    return primaryJoystick.getRawAxis(1);
  }
  
  public double getPrimaryZ(){
    return primaryJoystick.getRawAxis(5);
  }
   //-----------------------------------------------------secondary--------------------------------------------------------------------
  
  //Xbox Controller (Secondary)
  private static Joystick secondaryJoystick = new Joystick(1);

  public Joystick getSecondaryJoystick() {
    return secondaryJoystick;
  }
//-----------------------------------------------------------IDs CHANGE ALLLL RAAAHHHHHHHHH------------------------------------------------------------------------------

  //Swerve Drive
  public static final int DrivetrainSubsystem_FRONT_LEFT_DRIVE_MOTOR = 0; 
  public static final int DrivetrainSubsystem_FRONT_LEFT_ANGLE_MOTOR = 1; 

  public static final int DrivetrainSubsystem_BACK_LEFT_DRIVE_MOTOR = 2; 
  public static final int DrivetrainSubsystem_BACK_LEFT_ANGLE_MOTOR = 3; 

  //wired wrong so bad
  public static final int DrivetrainSubsystem_BACK_RIGHT_DRIVE_MOTOR = 5; 
  public static final int DrivetrainSubsystem_BACK_RIGHT_ANGLE_MOTOR = 4; 

  public static final int DrivetrainSubsystem_FRONT_RIGHT_DRIVE_MOTOR = 6; 
  public static final int DrivetrainSubsystem_FRONT_RIGHT_ANGLE_MOTOR = 7; 
  
  public static final HolonomicPathFollowerConfig pathFollowConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(0.5, 0.0, 0.1), 
      new PIDConstants(0.5, 0.0, 0.1), 
      maxSpeeds, 
      new Translation2d(0.6 / 2.0, 0.6 / 2.0).getNorm(), 
      new ReplanningConfig());
  
}