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
  private final CoralEndEffector m_endeff = new CoralEndEffector();
  private final CoralIntake m_intake = new CoralIntake();
  private final Climber m_climber = new Climber();

  //commands
  private final Command m_manual = new ElevatorManual(m_elevator);




  // Replace with CommandPS4Controller or CommandJoystick if needed

  CommandJoystick m_primaryJoystick = Setup.getInstance().getPrimaryJoystick();
  CommandXboxController m_secondary = Setup.getInstance().getSecondaryJoystick();
  public double speed = 0;
  //commands
  ClimberMove m_climbMan = new ClimberMove(m_climber);
  private final SequentialCommandGroup m_pickup = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorSnap(m_elevator,true,0),
        new EndToAngle(m_endeff, 0.0).withTimeout(5)),
      m_intake.Pivot(true),
      new intakeCommand(m_intake).withTimeout(10),
      m_endeff.pistonMove(true));
  //Driver speeds were here REMOVED FOR CLARITY
  
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   * REMOVED FOR CLARITY
   */

  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative input stream.
   * REMOVED FOR CLARITY
   */
  
  
   // Derive the heading axis with math! REMOVED FOR CLARITY


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //the below command makes elevator move to grabbing position, puts the grabby bit at the right angle, runs the intake, and opens grabby bit (we're using some sort of grabber right?)

    //default commands
    m_endeff.setDefaultCommand(new ParallelCommandGroup(
      m_endeff.stop(),
      m_endeff.pistonMove(false)));// stops movement and closes claw

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
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
    
    Setup.getInstance().toggleElevator.toggleOnTrue(new ElevatorSnap(m_elevator,false,-1));
    Setup.getInstance().toggleElevator.toggleOnFalse(m_manual);

    //DRIVETRAIN COMMAND ASSIGNMENTS REMOVED FOR SIMPLICITY

    //create triggers for primary buttons DRIVE ONES REMOVED FOR SIMPLICITY

   //TRIGGERS AND COMMANDS MATCHED, DRIVERS REMOVED FOR SIMPLICITY

    //m_elevator.setDefaultCommand(m_manual);
    m_secondary.leftBumper().onTrue(new SequentialCommandGroup(new EndToAngle(m_endeff, 0.0),new ElevatorSnap(m_elevator,true,0)));
    // automatically bring elevator to 0 if left bumper pressed, first ensure endeffector is in position
    
    //create secondary triggers
    BooleanSupplier spinIsPos = () -> Setup.getInstance().getSecondaryRX() >0.1;
    Trigger spinPosTrig = new Trigger(spinIsPos);
    BooleanSupplier spinIsNeg = () -> Setup.getInstance().getSecondaryRX() <-0.1;
    Trigger spinNegTrig = new Trigger(spinIsNeg);
    BooleanSupplier climbSelf = ()->Setup.getInstance().getClimbasBool();
    Trigger climbSelfTrig = new Trigger(climbSelf);
    BooleanSupplier climbManUp = ()->Setup.getInstance().getRightJoyIsPos();
    Trigger climbManUpTrig = new Trigger(climbManUp);
    BooleanSupplier climbManDown = ()->Setup.getInstance().getRightJoyIsNeg();
    Trigger climbManDownTrig = new Trigger(climbManDown);

    //COMMAND/TRIGGER ASSIGNMENTS, DRIVER RELATED REMOVED FROM CLIMBER FOR CLARITY
    m_secondary.start().toggleOnTrue(m_climbMan);
    m_secondary.start().toggleOnFalse(m_manual);
    climbSelfTrig.onTrue(m_climber.ClimbSelf());
    m_secondary.a().onTrue(new EndToAngle(m_endeff,0.0));
    m_secondary.b().onTrue(new EndToAngle(m_endeff,35.0));
    m_secondary.x().onTrue(new EndToAngle(m_endeff,35.0));
    m_secondary.y().onTrue(new EndToAngle(m_endeff,179.0));
    spinPosTrig.whileTrue(m_endeff.spinClockwise());
    spinNegTrig.whileTrue(m_endeff.spinCounterClockwise());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drivebase);
  }
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
