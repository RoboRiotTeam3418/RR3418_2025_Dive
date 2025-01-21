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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;

import java.io.File;
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
  //private final SwerveSubsystem m_drivetrain = new SwerveSubsystem();

  //commands
  private final Command m_climb = new ClimberMove(m_climber);
  private final Command m_snap = new ElevatorSnap(m_elevator);
  private final Command m_manual = new ElevatorManual(m_elevator);
  //private final Command m_simpDrive = new simpleDriveCommand(m_drivetrain);



  // Replace with CommandPS4Controller or CommandJoystick if needed

  CommandJoystick m_primaryJoystick = Setup.getInstance().getPrimaryJoystick();
  CommandXboxController m_secondary = Setup.getInstance().getSecondaryJoystick();

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_primaryJoystick.getY() * -1,// CHECK FUNCTION
                                                                () -> m_primaryJoystick.getX() * -1)// CHECK FUNCTION
                                                            .withControllerRotationAxis(m_primaryJoystick::getZ)// CHECK FUNCTION
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_primaryJoystick::getZ)
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
    m_elevator.setDefaultCommand(m_manual);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));*/

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    Setup.getInstance().toggleClimber.toggleOnTrue(m_climb);
    Setup.getInstance().toggleElevator.toggleOnTrue(m_snap);
    Setup.getInstance().toggleElevator.toggleOnFalse(m_manual);
    //m_drivetrain.setDefaultCommand(m_simpDrive);


    Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
    Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleSim);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      m_primaryJoystick.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_primaryJoystick.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      m_primaryJoystick.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_primaryJoystick.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      m_primaryJoystick.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_primaryJoystick.back().whileTrue(drivebase.centerModulesCommand());
      m_primaryJoystick.leftBumper().onTrue(Commands.none());
      m_primaryJoystick.rightBumper().onTrue(Commands.none());
    } else
    {
      m_primaryJoystick.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_primaryJoystick.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      m_primaryJoystick.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      m_primaryJoystick.start().whileTrue(Commands.none());
      m_primaryJoystick.back().whileTrue(Commands.none());
      m_primaryJoystick.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_primaryJoystick.rightBumper().onTrue(Commands.none());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
