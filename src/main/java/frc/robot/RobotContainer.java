// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.swervedrive.drivebase.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;

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

  // Replace with CommandPS4Controller or CommandJoystick if needed

  CommandJoystick m_primaryJoystick = Setup.getInstance().getPrimaryJoystick();
  //CommandXboxController m_secondary = Setup.getInstance().getSecondaryJoystick();
  //Driver speeds

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
    // limit acceleration
   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  //DoubleSupplier rotSupplier = () -> drivebase.getRot(m_primaryJoystick.getTwist());
  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  //SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_primaryJoystick::getTwist, getNegTwist)//checkfunction
                                                           //.headingWhile(true);
  /*  Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocity.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    m_primaryJoystick.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    m_primaryJoystick.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);*/


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    //m_elevator.setDefaultCommand(m_manual);
    //m_endeff.setDefaultCommand(Commands.none());
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


    //Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocityXtraSlow    = drivebase.driveFieldOriented(drivebase.getAngularVelocity(drivebase,"xtraSlow"));
    Command driveFieldOrientedAnglularVelocitySlow    = drivebase.driveFieldOriented(drivebase.getAngularVelocity(drivebase,"slow"));
    Command driveFieldOrientedAnglularVelocityMed    = drivebase.driveFieldOriented(drivebase.getAngularVelocity(drivebase,"medium"));
    Command driveFieldOrientedAnglularVelocityFast   = drivebase.driveFieldOriented(drivebase.getAngularVelocity(drivebase,"fast"));
    //Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    //Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
    //Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
        //driveDirectAngleSim);
    final Supplier<ChassisSpeeds> DEATH_SPEEDS = () -> new ChassisSpeeds(0,0, drivebase.getSwerveDrive().getMaximumChassisAngularVelocity());
    //Command death = drivebase.drive(DEATH_SPEEDS);

    //create triggers for primary buttons
    BooleanSupplier fullStop = () ->Setup.getInstance().getFullStop(); 
    Trigger fullStopTrig = new Trigger(fullStop);
    BooleanSupplier zeroGyro = () ->Setup.getInstance().getZeroGyro(); 
    Trigger zeroGyroTrig = new Trigger(zeroGyro);
    BooleanSupplier primaryStart = () ->Setup.getInstance().getPrimaryStart(); 
    Trigger primaryStartTrig = new Trigger(primaryStart);
    BooleanSupplier primaryBack = () ->Setup.getInstance().getPrimaryBack(); 
    Trigger primaryBackTrig = new Trigger(primaryBack);
    /* 
    BooleanSupplier backIsPos = () ->Setup.getInstance().getBackIsPos();
    Trigger backIsPosTrig = new Trigger(backIsPos);
    BooleanSupplier backIsNeg = () ->Setup.getInstance().getBackIsNeg();
    Trigger backIsNegTrig = new Trigger(backIsNeg);*/
    BooleanSupplier driveSetDistance = () ->Setup.getInstance().getDriveSetDistance();
    Trigger driveSetDistanceTrig = new Trigger(driveSetDistance);
    BooleanSupplier fakeVision = () ->Setup.getInstance().getFakeVision();
    Trigger fakeVisionTrig = new Trigger(fakeVision);
    BooleanSupplier deathMode = () -> Setup.getInstance().getDeathMode();
    Trigger deathModeTrig = new Trigger(deathMode);
    BooleanSupplier xtraSlow = () -> Setup.getInstance().getPrimaryDriverXButton();
    Trigger xtraSlowTrig = new Trigger(xtraSlow);
    BooleanSupplier slow = () -> Setup.getInstance().getPrimaryDriverAButton();
    Trigger slowTrig = new Trigger(slow);
    BooleanSupplier medium = () -> Setup.getInstance().getPrimaryDriverBButton();
    Trigger mediumTrig = new Trigger(medium);
    BooleanSupplier fast = () -> Setup.getInstance().getPrimaryDriverYButton();
    Trigger fastTrig = new Trigger(fast);
        /*if (RobotBase.isAutonomous()){
                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }else{*/
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityMed);
      xtraSlowTrig.onTrue(driveFieldOrientedAnglularVelocityXtraSlow);
      slowTrig.onTrue(driveFieldOrientedAnglularVelocitySlow);
      mediumTrig.onTrue(driveFieldOrientedAnglularVelocityMed);
      fastTrig.onTrue(driveFieldOrientedAnglularVelocityFast);
       
      /*zeroGyroTrig.onTrue((Commands.runOnce(drivebase::zeroGyro)));
      fakeVisionTrig.onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driveSetDistanceTrig.whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      primaryStartTrig.whileTrue(Commands.none());
      primaryBackTrig.whileTrue(Commands.none());*/
      //backIsNegTrig.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //backIsPosTrig.onTrue(Commands.none());
    //zeroGyroTrig.onTrue((Commands.runOnce(drivebase::zeroGyro)));
    /* 
    deathModeTrig.onTrue(Commands.none());
    deathModeTrig.onFalse(driveFieldOrientedAnglularVelocity);
    if(deathModeTrig.getAsBoolean()){
      drivebase.drive(DEATH_SPEEDS.get());
      
    }*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(drivebase);
  }
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
