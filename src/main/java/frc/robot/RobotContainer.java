// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
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
  private final Climber m_climber = new Climber();

  CommandJoystick m_primaryJoystick = Setup.getInstance().getPrimaryJoystick();
  CommandXboxController m_secondary = Setup.getInstance().getSecondaryJoystick();
  //commands
  private final Command m_manual = new ElevatorManual(m_elevator);
  private final Command m_climberMove = new ClimberMove(m_climber, m_secondary);

  public double speed = 0;
  //commands
  ClimberMove m_climbMan = new ClimberMove(m_climber,m_secondary);
  private final SequentialCommandGroup m_pickup = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorSnap(m_elevator,true,0),
        new EndToAngle(m_endeff, 0.0).withTimeout(20)),
      m_endeff.pistonMove(true));

  //Driver speeds
  
  private final SwerveSubsystem  drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

    /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> m_primaryJoystick.getY() ,// CHECK FUNCTION
  () -> m_primaryJoystick.getX())// CHECK FUNCTION
.withControllerRotationAxis(m_primaryJoystick::getTwist)// CHECK FUNCTION
.deadband(OperatorConstants.DEADBAND)
.scaleTranslation(0.8)
.allianceRelativeControl(true);

/**
* Clones the angular velocity input stream and converts it to a fieldRelative input stream.
*/
public DoubleSupplier getNegTwist = ()-> m_primaryJoystick.getTwist()*-1;
SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_primaryJoystick::getTwist, getNegTwist)//checkfunction
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
    
    //the below command makes elevator move to grabbing position, puts the grabby bit at the right angle,  and opens grabby bit (we're using some sort of grabber right?)

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
    //DRIVETRAIN COMMAND ASSIGNMENTS 
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
    BooleanSupplier fullStop = () ->Setup.getInstance().getFullStop(); 
    Trigger fullStopTrig = new Trigger(fullStop);
    BooleanSupplier zeroGyro = () ->Setup.getInstance().getZeroGyro(); 
    Trigger zeroGyroTrig = new Trigger(zeroGyro);
    BooleanSupplier primaryStart = () ->Setup.getInstance().getPrimaryStart(); 
    Trigger primaryStartTrig = new Trigger(primaryStart);
    BooleanSupplier primaryBack = () ->Setup.getInstance().getPrimaryBack(); 
    Trigger primaryBackTrig = new Trigger(primaryBack);
    BooleanSupplier backIsPos = () ->Setup.getInstance().getBackIsPos();
    Trigger backIsPosTrig = new Trigger(backIsPos);
    BooleanSupplier backIsNeg = () ->Setup.getInstance().getBackIsNeg();
    Trigger backIsNegTrig = new Trigger(backIsNeg);
    BooleanSupplier driveSetDistance = () ->Setup.getInstance().getDriveSetDistance();
    Trigger driveSetDistanceTrig = new Trigger(driveSetDistance);
    BooleanSupplier fakeVision = () ->Setup.getInstance().getFakeVision();
    Trigger fakeVisionTrig = new Trigger(fakeVision);
    BooleanSupplier deathMode = () -> Setup.getInstance().getDeathMode();
    Trigger deathModeTrig = new Trigger(deathMode);

   // automatically bring elevator to 0 if left bumper pressed, first ensure endeffector is in position
    
    //create secondary triggers
    BooleanSupplier spinIsPos = () -> Setup.getInstance().getSecondaryRX() >0.1;
    Trigger spinPosTrig = new Trigger(spinIsPos);
    BooleanSupplier spinIsNeg = () -> Setup.getInstance().getSecondaryRX() <-0.1;
    Trigger spinNegTrig = new Trigger(spinIsNeg);
    BooleanSupplier climbSelf = ()->Setup.getInstance().getClimbasBool();
    Trigger climbSelfTrig = new Trigger(climbSelf);

    BooleanSupplier climbNotSched = () -> !m_climberMove.isScheduled();

    //COMMAND/TRIGGER ASSIGNMENTS
    m_secondary.start().toggleOnTrue(new ParallelCommandGroup(m_climberMove, new SequentialCommandGroup(new EndToAngle(m_endeff, 0.0),new ElevatorSnap(m_elevator,true,0))));
    m_secondary.start().toggleOnFalse(m_manual);
    climbSelfTrig.onTrue(m_climber.ClimbSelf());
    

    Setup.getInstance().toggleElevator.and(climbNotSched).toggleOnTrue(new ElevatorSnap(m_elevator,false,-1));
    Setup.getInstance().toggleElevator.and(climbNotSched).toggleOnFalse(m_manual);


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
    zeroGyroTrig.onTrue((Commands.runOnce(drivebase::zeroGyro)));
    deathModeTrig.whileTrue(death);

    m_secondary.a().onTrue(new EndToAngle(m_endeff,0.0));
    m_secondary.b().onTrue(new EndToAngle(m_endeff,35.0));
    m_secondary.y().onTrue(new EndToAngle(m_endeff,90.0));
    spinPosTrig.whileTrue(m_endeff.spinClockwise());
    spinNegTrig.whileTrue(m_endeff.spinCounterClockwise());
    m_secondary.leftBumper().and(climbNotSched).onTrue(new SequentialCommandGroup(new EndToAngle(m_endeff, 0.0),new ElevatorSnap(m_elevator,true,0)));

    Setup.getInstance().toggleElevator.and(climbNotSched).toggleOnTrue(new ElevatorSnap(m_elevator,false,-1));
    Setup.getInstance().toggleElevator.and(climbNotSched).toggleOnFalse(m_manual);


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
    zeroGyroTrig.onTrue((Commands.runOnce(drivebase::zeroGyro)));
    deathModeTrig.whileTrue(death);

    m_secondary.a().onTrue(new EndToAngle(m_endeff,0.0));
    m_secondary.b().onTrue(new EndToAngle(m_endeff,35.0));
    m_secondary.y().onTrue(new EndToAngle(m_endeff,90.0));
    spinPosTrig.whileTrue(m_endeff.spinClockwise());
    spinNegTrig.whileTrue(m_endeff.spinCounterClockwise());
    m_secondary.leftBumper().and(climbNotSched).onTrue(new SequentialCommandGroup(new EndToAngle(m_endeff, 0.0),new ElevatorSnap(m_elevator,true,0)));

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
