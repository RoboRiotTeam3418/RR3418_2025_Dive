// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Elevator m_elevator = new Elevator();
  private final Climber m_climber = new Climber();
  private final CoralIntake m_Intake = new CoralIntake();
  private final Example_Subsystem m_exampleSubsystem = new Example_Subsystem();
  private final CoralEndEffector m_endEffect = new CoralEndEffector();
  //private final SwerveSubsystem m_drivetrain = new SwerveSubsystem();

  //commands
  private final Command m_climb = new ClimberMove(m_climber);
  //private final Command m_snap = new ElevatorSnap(m_elevator);
  private final Command m_manual = new ElevatorManual(m_elevator);

  //the below command makes elevator move to grabbing position, puts the grabby bit at the right angle, runs the intake, and opens grabby bit (we're using some sort of grabber right?)
  private final SequentialCommandGroup m_pickup = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new ElevatorSnap(m_elevator,0),
      new SetArmCommand(m_endEffect, 0, 10),
      m_endEffect.pistonMove(true)
    ),
    new intakeCommand(.7, m_Intake, true));
  //private final Command m_simpDrive = new simpleDriveCommand(m_drivetrain);



  // Replace with CommandPS4Controller or CommandJoystick if needed

  CommandJoystick m_primaryJoystick = Setup.getInstance().getPrimaryJoystick();
  CommandXboxController m_secondary = Setup.getInstance().getSecondaryJoystick();

  //Driver speeds
  public Double getSpeedSetting(){
    //determine which speed setting the driver sets
      if(Setup.getInstance().getDeathMode()){
              speedSetting = "death";
      } else if(Setup.getInstance().getPrimaryDriverXButton()){
              speedSetting = "fast";
      } else if(Setup.getInstance().getPrimaryDriverAButton()){
              speedSetting = "medium";
      } else if(Setup.getInstance().getPrimaryDriverBButton()){
              speedSetting = "slow";
      } else if(Setup.getInstance().getPrimaryDriverYButton()){
              speedSetting = "reallySlow";
      }
  //set the speed based on the current speed setting
      String whichSpeed = speedSetting;
      if(whichSpeed == "death"){
              speed =Constants.MAX_SPEED;
      } else if(whichSpeed == "fast"){
              speed=-.825;
      } else if(whichSpeed == "medium"){
              speed=0;
      } else if(whichSpeed == "slow"){
              speed=0.325;
      } else if(whichSpeed == "reallySlow"){
              speed = .5;
      }
      return speed;
    }
    public String speedSetting = "medium";
    public double speed = 0;

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_primaryJoystick.getY() * -1*speed,// CHECK FUNCTION
                                                                () -> m_primaryJoystick.getX() * -1*speed)// CHECK FUNCTION
                                                            .withControllerRotationAxis(m_primaryJoystick::getZ)// CHECK FUNCTION
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  public DoubleSupplier getNegZ = ()-> m_primaryJoystick.getZ()*-1;
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_primaryJoystick::getZ, getNegZ)//checkfunction
                                                           .headingWhile(true);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -m_primaryJoystick.getY(),
                                                                   () -> -m_primaryJoystick.getX())
                                                               .withControllerRotationAxis(() -> m_primaryJoystick.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    m_primaryJoystick.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    m_primaryJoystick.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    //default commands
    m_elevator.setDefaultCommand(m_manual);
    m_endEffect.setDefaultCommand(new ParallelCommandGroup(
      m_endEffect.stop(),
      m_endEffect.pistonMove(false)
    ));//stops movement of motors and closes claw by default

    //end effector commands
    m_secondary.y().whileTrue(new SetArmCommand(m_endEffect, 0, 10));
    m_secondary.leftBumper().whileTrue(m_endEffect.spin(-1));
    m_secondary.rightBumper().whileTrue(m_endEffect.spin(1));
    m_secondary.b().whileTrue(new SetArmCommand(m_endEffect, 35, 10));
    m_secondary.rightTrigger().whileTrue(m_endEffect.pistonMove(true));

    //elevator commands
    m_secondary.a().onTrue(m_elevator.setSnap()).whileTrue(m_manual);
    m_secondary.leftTrigger().whileTrue(new ElevatorSnap(m_elevator, m_elevator.goalheight));

    //the below command makes elevator move to grabbing position, puts the grabby bit at the right angle, runs the intake, and opens grabby bit (we're using some sort of grabber right?)
    m_primaryJoystick.trigger().whileTrue(m_pickup);

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("Place", new SequentialCommandGroup(
      new ElevatorSnap(m_elevator, 3), 
      new SetArmCommand(m_endEffect, 35, 10), 
      m_endEffect.pistonMove(true)
    ));
    NamedCommands.registerCommand("pickup", m_pickup);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));*/

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //Setup.getInstance().toggleClimber.toggleOnTrue(m_climb);
    
    /*Setup.getInstance().toggleElevator.toggleOnTrue(m_snap);
    Setup.getInstance().toggleElevator.toggleOnFalse(m_manual);*/
    //m_drivetrain.setDefaultCommand(m_simpDrive);


    Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
    Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleSim);
    final Supplier<ChassisSpeeds> DEATH_SPEEDS = () -> new ChassisSpeeds(0,0, drivebase.getSwerveDrive().getMaximumChassisAngularVelocity());
    Command death = drivebase.drive(DEATH_SPEEDS);

    //create triggers for primary buttons
    BooleanSupplier fullStop = () ->Setup.getInstance().getFullStop(); //3
    Trigger fullStopTrig = new Trigger(fullStop);
    BooleanSupplier zeroGyro = () ->Setup.getInstance().getZeroGyro(); //2
    Trigger zeroGyroTrig = new Trigger(zeroGyro);
    BooleanSupplier primaryStart = () ->Setup.getInstance().getPrimaryStart(); //11 (the two sqares)
    Trigger primaryStartTrig = new Trigger(primaryStart);
    BooleanSupplier primaryBack = () ->Setup.getInstance().getPrimaryBack(); //12 (the three lines)
    Trigger primaryBackTrig = new Trigger(primaryBack);
    BooleanSupplier backIsPos = () ->Setup.getInstance().getBackIsPos(); //slider
    Trigger backIsPosTrig = new Trigger(backIsPos);
    BooleanSupplier backIsNeg = () ->Setup.getInstance().getBackIsNeg(); //slider
    Trigger backIsNegTrig = new Trigger(backIsNeg);
    BooleanSupplier driveSetDistance = () ->Setup.getInstance().getDriveSetDistance(); //15
    Trigger driveSetDistanceTrig = new Trigger(driveSetDistance);
    BooleanSupplier fakeVision = () ->Setup.getInstance().getFakeVision(); //15
    Trigger fakeVisionTrig = new Trigger(fakeVision);
    BooleanSupplier deathMode = () -> Setup.getInstance().getDeathMode(); //10
    Trigger deathModeTrig = new Trigger(deathMode);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
     primaryStartTrig.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_primaryJoystick.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      fullStopTrig.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driveSetDistanceTrig.whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      zeroGyroTrig.onTrue((Commands.runOnce(drivebase::zeroGyro)));
      primaryBackTrig.whileTrue(drivebase.centerModulesCommand());
      backIsNegTrig.onTrue(Commands.none());
      backIsPosTrig.onTrue(Commands.none());
    } else
    {
      zeroGyroTrig.onTrue((Commands.runOnce(drivebase::zeroGyro)));
      fakeVisionTrig.onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driveSetDistanceTrig.whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      primaryStartTrig.whileTrue(Commands.none());
      primaryBackTrig.whileTrue(Commands.none());
      backIsNegTrig.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      backIsPosTrig.onTrue(Commands.none());
      deathModeTrig.whileTrue(death);
      
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("New Auto");
  }
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
