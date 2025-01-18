package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.util.drivers.Gyroscope;
import frc.robot.util.drivers.NavX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
public class Setup {
    
  public static Setup instance = new Setup();

  public static Setup getInstance() {
    if (instance == null) {
      instance = new Setup();
    }
	  return instance;
  }   

//---------------------------------------------------------Driver Config------------------------------------------------------------------
/* OLD AND BROKEN
  public SparkMaxConfig config = new SparkMaxConfig();
  config
    .inverted(false)
    .idleMode(IdleMode.kBrake);
  config.closedLoop
    .pid(constants.p, constants.i, constants.d);
  config.closedLoop.maxMotion
    .maxVelocity(Setup.maxVel)
    .maxAcceleration(Setup.maxAccel);
  config.encoder
    .positionConversionFactor(wheelDiameter * Math.PI / reduction)
    .velocityConversionFactor(wheelDiameter * Math.PI / reduction * (1.0 / 60.0)); // RPM to units per second);
  public SparkMaxConfig getinstance() {
    return config;
  }*/


 //---------------------------------------------------------Swerve Drive------------------------------------------------------------------

  //measurments of robot in meters from center of wheel (23 inches squared, 39.37 inches in a meter)
  public double TRACKWIDTH = 39.37/23;
  public double WHEELBASE = TRACKWIDTH;
    
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

  //Acceleration and velocity max for pid config VALUES CURRENTLY BASED ON NOTHING
  public static final double maxVel = 0.5;
  public static final double maxAccel = 0.2;


 //----------------------------------------------------------Primary----------------------------------------------------------------------
//FOR COMMAND JOYSTICKS MAKE BUTTONS HERE AND PUBLIC, MAKE DOUBLES PUBLIC METHODS SO IT GETS EACH TIME IT'S CALLED THIS WAY YOU DON"T HAVE 
//TO REFERENCE THE JOYSTICK OBJECT ITSELF EACH TIME

  //Flight Stick (Primary)
  private static CommandJoystick primaryJoystick = new CommandJoystick(0);

  public CommandJoystick getPrimaryJoystick() {
    return primaryJoystick;
  }
  
  //movement
  public double getPrimaryX(){
    return primaryJoystick.getX();
  }
  
  public double getPrimaryY(){
    return primaryJoystick.getY();
  }
  
  public double getPrimaryZ(){
    return primaryJoystick.getZ();
  }
   //-----------------------------------------------------secondary--------------------------------------------------------------------
  
  //Xbox Controller (Secondary)
  private static CommandXboxController secondaryJoystick = new CommandXboxController(1);
  public final Trigger toggleClimber = secondaryJoystick.start();
  public final Trigger toggleElevator = secondaryJoystick.back();

  public CommandXboxController getSecondaryJoystick() {
    return secondaryJoystick;
  }
  public boolean getSecondaryAasBool(){
    return secondaryJoystick.getHID().getAButtonPressed();
  }

  public boolean getSecondaryMoveElev(){
    return secondaryJoystick.getHID().getLeftStickButtonPressed();
  }

  public Double getSecondaryLY(){
    return secondaryJoystick.getLeftY();
  }
//---------------------------------------------------------Hardware------------------------------------------------------------------------

  //Gyroscope
  public final Gyroscope gyroscope = NavX.getInstance();
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

  public static final int ELEVMOT1ID = 7; 
  public static final int ELEVMOT2ID = 7; 

  public static final int CLIMB_ID = 7; 
  public static final int INTAKE_TOP_ID = 7; 
  public static final int INTAKE_BOT_ID = 7; 
  
}