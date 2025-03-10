package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.math.DeadbandUtils;

public class Setup {

  public static Setup instance = new Setup();

  public static Setup getInstance() {
    if (instance == null) {
      instance = new Setup();
    }
    return instance;
  }

  // ---------------------------------------------------------Swerve
  // Drive------------------------------------------------------------------

  // measurments of robot in meters from center of wheel (23 inches squared, 39.37
  // inches in a meter)
  public double TRACKWIDTH = 39.37 / 23;
  public double WHEELBASE = TRACKWIDTH;

  /*
   * offset of wheels sets the angle to start - CHANGE DIS BRO
   * public double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(123.579545);
   * public double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(260.795455);
   * public double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(-4.303369);
   * public double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(18.579545);
   */

  // finds position of the wheels based on the position of the center
  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
      new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
      new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
      new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0));

  // Acceleration and velocity max for pid config VALUES CURRENTLY BASED ON
  // NOTHING
  public static final double maxVel = 0.5;
  public static final double maxAccel = 0.2;

  // ----------------------------------------------------------Primary----------------------------------------------------------------------
  // FOR COMMAND JOYSTICKS MAKE BUTTONS HERE AND PUBLIC, MAKE DOUBLES PUBLIC
  // METHODS SO IT GETS EACH TIME IT'S CALLED THIS WAY YOU DON"T HAVE
  // TO REFERENCE THE JOYSTICK OBJECT ITSELF EACH TIME

  // Flight Stick (Primary)
  private static CommandJoystick primaryJoystick = new CommandJoystick(0);

  public CommandJoystick getPrimaryJoystick() {
    return primaryJoystick;
  }

  public Joystick getPrimaryHID() {
    return primaryJoystick.getHID();
  }

  // movement
  public double getPrimaryX() {
    return primaryJoystick.getX();
  }

  public double getPrimaryY() {
    return primaryJoystick.getY();
  }

  public double getPrimaryZ() {
    return primaryJoystick.getZ();
  }

  // speed
  public boolean getPrimaryDriverXButton() {
    return getPrimaryHID().getRawButton(5);
  }

  public boolean getPrimaryDriverAButton() {
    return getPrimaryHID().getRawButton(6);
  }

  public boolean getPrimaryDriverBButton() {
    return getPrimaryHID().getRawButton(7);
  }

  public boolean getPrimaryDriverYButton() {
    return getPrimaryHID().getRawButton(8);
  }

  public boolean getDeathMode() {
    return getPrimaryHID().getRawButton(10);
  }

  // field oriented
  public boolean getFieldOriented() {
    return getPrimaryHID().getRawButtonPressed(4);
  }

  // other drivy things
  public boolean getFullStop() {
    return getPrimaryHID().getRawButtonPressed(3);
  }

  public boolean getZeroGyro() {
    return getPrimaryHID().getRawButtonPressed(2);
  }

  public boolean getPrimaryBack() {
    // center modules in test, nothing in Teleop
    return getPrimaryHID().getRawButtonPressed(11);
  }

  public boolean getPrimaryStart() {
    // reset odometry in simulation, nothing in teleop
    return getPrimaryHID().getRawButtonPressed(12);
  }

  public boolean getBackIsPos() {
    // clears command
    return getPrimaryHID().getRawAxis(6) > 0.1;
  }

  public boolean getBackIsNeg() {
    // stops driver, clears command
    return getPrimaryHID().getRawAxis(6) < -0.1;
  }

  public boolean getDriveSetDistance() {
    return getPrimaryHID().getRawButtonPressed(15);
  }

  public boolean getFakeVision() {
    return getPrimaryHID().getRawButtonPressed(13);
  }

  // -----------------------------------------------------secondary--------------------------------------------------------------------

  // Xbox Controller (Secondary)
  private static CommandXboxController secondaryJoystick = new CommandXboxController(1);
  /*
   * public final Trigger toggleClimber = secondaryJoystick.start();
   * public final Trigger POVUp = new POVButton(secondaryJoystick.getHID(), 0);
   * public final Trigger POVDown = new POVButton(secondaryJoystick.getHID(),
   * 180);
   */

  public CommandXboxController getSecondaryJoystick() {
    return secondaryJoystick;
  }

  public Double getSecondaryRX() {
    // spins Endeff
    return secondaryJoystick.getRightX();
  }

  public boolean getSecondaryMoveElev() {
    return secondaryJoystick.rightBumper().getAsBoolean();
  }

  public Double getSecondaryLY() {
    return secondaryJoystick.getLeftY();
  }

  public boolean getClimbasBool() {
    return secondaryJoystick.getHID().getLeftStickButtonPressed();
  }

  public boolean getRightJoyIsOn() {
    return DeadbandUtils.isGreater(secondaryJoystick.getRightTriggerAxis(), 0.1);
  }

  /*
   * public boolean getSecondaryPOVUpasBool() {
   * return POVUp.getAsBoolean();
   * }
   * 
   * public boolean getSecondaryPOVDownasBool() {
   * return POVDown.getAsBoolean();
   * }
   */

  // ---------------------------------------------------------Hardware------------------------------------------------------------------------

  // -----------------------------------------------------------IDs CHANGE
  // RAAAHHHHHHHHH------------------------------------------------------------------------------

  // Swerve Drive
  public static final int DrivetrainSubsystem_FRONT_LEFT_DRIVE_MOTOR = 10;
  public static final int DrivetrainSubsystem_FRONT_LEFT_ANGLE_MOTOR = 11;

  public static final int DrivetrainSubsystem_BACK_LEFT_DRIVE_MOTOR = 12;
  public static final int DrivetrainSubsystem_BACK_LEFT_ANGLE_MOTOR = 13;

  // wired wrong so bad
  public static final int DrivetrainSubsystem_BACK_RIGHT_DRIVE_MOTOR = 14;
  public static final int DrivetrainSubsystem_BACK_RIGHT_ANGLE_MOTOR = 15;

  public static final int DrivetrainSubsystem_FRONT_RIGHT_DRIVE_MOTOR = 16;
  public static final int DrivetrainSubsystem_FRONT_RIGHT_ANGLE_MOTOR = 17;

  public static final int ELEVMOT1ID = 20;
  public static final int ELEVMOT2ID = 21;

  public static final int CLIMB1_ID = 24;
  public static final int CLIMB2_ID = 23;
  // public static final int SPIN_ID = 22;
  public static final int GamePieceSensorID = 0;

}