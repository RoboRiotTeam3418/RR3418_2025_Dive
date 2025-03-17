// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.drivers.Limelight;
import frc.robot.util.drivers.Toggles;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm();
  private final Claw m_claw = new Claw();
  private final Limelight m_Limelight = new Limelight();

  CommandJoystick m_primaryJoystick = Setup.getInstance().getPrimaryJoystick();
  CommandXboxController m_secondary = Setup.getInstance().getSecondaryJoystick();
  // commands
  private final Command m_elevManual = new ElevatorManual(m_elevator);
  
  private final Command m_armManual = new ArmManual(m_arm);
  private final Command m_readyClaw = new ReadyClaw(m_claw);
  
  //public double speed = 0;
  // commands
  private final SequentialCommandGroup m_pickup = new SequentialCommandGroup(
      m_elevator.setSnap(ElevatorLevel.LOWEST),
      new ParallelCommandGroup(
          new ElevatorSnap(m_elevator)));
  // new EndToAngle(m_endeff, 0.0).withTimeout(20)),
  // m_endeff.pistonMove(true));

  // Driver speeds

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  public DoubleSupplier getPosTwist = () -> m_primaryJoystick.getRawAxis(5);
  public double speed = 0.5, xtraSlow = 0.35, slow = 0.5, med = 0.75, fast = 0.8;
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_primaryJoystick.getY(), // CHECK FUNCTION
      () -> m_primaryJoystick.getX())// CHECK FUNCTION
      .withControllerRotationAxis(()->m_primaryJoystick.getRawAxis(5))// CHECK FUNCTION
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  SwerveInputStream driveAngularVelocityXtraSlow = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_primaryJoystick.getY() * xtraSlow, // CHECK FUNCTION
      () -> m_primaryJoystick.getX() * xtraSlow)// CHECK FUNCTION
      .withControllerRotationAxis(getPosTwist)
      .deadband(OperatorConstants.DEADBAND)
      // .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  SwerveInputStream driveAngularVelocitySlow = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_primaryJoystick.getY() * slow, // CHECK FUNCTION
      () -> m_primaryJoystick.getX() * slow)// CHECK FUNCTION
      .withControllerRotationAxis(getPosTwist)
      .deadband(OperatorConstants.DEADBAND)
      // .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  SwerveInputStream driveAngularVelocityMed = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_primaryJoystick.getY() * med, // CHECK FUNCTION
      () -> m_primaryJoystick.getX() * med)// CHECK FUNCTION
      .withControllerRotationAxis(getPosTwist)
      .deadband(OperatorConstants.DEADBAND)
      // .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  SwerveInputStream driveAngularVelocityFast = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_primaryJoystick.getY() * fast, // CHECK FUNCTION
      () -> m_primaryJoystick.getX() * fast)// CHECK FUNCTION
      .withControllerRotationAxis(getPosTwist)
      .deadband(OperatorConstants.DEADBAND)
      // .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  public DoubleSupplier getNegTwist = () -> m_primaryJoystick.getTwist() * -1;
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(m_primaryJoystick::getTwist, getNegTwist)// checkfunction
      .headingWhile(true);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -m_primaryJoystick.getY(),
      () -> -m_primaryJoystick.getX())
      .withControllerRotationAxis(() -> m_primaryJoystick.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          m_primaryJoystick.getRawAxis(
              2) * Math.PI)
          * (Math.PI * 2),
          () -> Math.cos(
              m_primaryJoystick.getRawAxis(
                  2) * Math.PI)
              *
              (Math.PI * 2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // DRIVETRAIN COMMAND ASSIGNMENTS R
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
    Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleSim);
    final Supplier<ChassisSpeeds> DEATH_SPEEDS = () -> drivebase.getDeath();

    // create triggers for primary buttons
    BooleanSupplier fullStop = () -> Setup.getInstance().getFullStop();
    Trigger fullStopTrig = new Trigger(fullStop);
    /*BooleanSupplier zeroGyro = () -> Setup.getInstance().getZeroGyro();
    Trigger zeroGyroTrig = new Trigger(zeroGyro);
    BooleanSupplier primaryStart = () -> Setup.getInstance().getPrimaryStart();
    Trigger primaryStartTrig = new Trigger(primaryStart);
    BooleanSupplier primaryBack = () -> Setup.getInstance().getPrimaryBack();
    Trigger primaryBackTrig = new Trigger(primaryBack);
    BooleanSupplier backIsPos = () -> Setup.getInstance().getBackIsPos();
    Trigger backIsPosTrig = new Trigger(backIsPos);
    BooleanSupplier backIsNeg = () -> Setup.getInstance().getBackIsNeg();
    Trigger backIsNegTrig = new Trigger(backIsNeg);
    BooleanSupplier driveSetDistance = () -> Setup.getInstance().getDriveSetDistance();
    Trigger driveSetDistanceTrig = new Trigger(driveSetDistance);
    BooleanSupplier fakeVision = () -> Setup.getInstance().getFakeVision();
    Trigger fakeVisionTrig = new Trigger(fakeVision);*/
    BooleanSupplier deathMode = () -> Setup.getInstance().getDeathMode();
    Trigger deathModeTrig = new Trigger(deathMode);

    // create secondary triggers
    Trigger m_spinPos = Setup.getInstance().spinPosTrig;
    Trigger m_spinNeg = Setup.getInstance().spinNegTrig;
    Trigger m_elevUp = Setup.getInstance().elevUpTrig;
    Trigger m_elevDown = Setup.getInstance().elevDownTrig;
    Trigger m_coralRelease = Setup.getInstance().releaseCoral;
    Trigger m_elevGo = Setup.getInstance().snapGo;
    //speed changer
    BooleanSupplier xtraSlow = () -> Setup.getInstance().getPrimaryDriverYButton();
    Trigger xtraSlowTrig = new Trigger(xtraSlow);
    BooleanSupplier slow = () -> Setup.getInstance().getPrimaryDriverBButton();
    Trigger slowTrig = new Trigger(slow);
    BooleanSupplier medium = () -> Setup.getInstance().getPrimaryDriverAButton();
    Trigger mediumTrig = new Trigger(medium);
    BooleanSupplier fast = () -> Setup.getInstance().getPrimaryDriverXButton();
    Trigger fastTrig = new Trigger(fast);
    BooleanSupplier spinIsPos = () -> Setup.getInstance().getSecondaryRX() > 0.1;
    Trigger spinPosTrig = new Trigger(spinIsPos);
    BooleanSupplier spinIsNeg = () -> Setup.getInstance().getSecondaryRX() < -0.1;
    Trigger spinNegTrig = new Trigger(spinIsNeg);
    BooleanSupplier spinIsOn = () -> spinIsPos.getAsBoolean()|| spinIsNeg.getAsBoolean();
    Trigger spinIsOnTrig = new Trigger(spinIsOn);

    // Default commands
    //WHAT DOES THIS DO?
    /*m_secondary.start().onTrue(new InstantCommand(new Runnable() {
      public void run() {
        Toggles.toggleSecondary();
      };
    }));*/

    // Elevator
    m_elevator.setDefaultCommand(m_elevManual);
    m_secondary.leftTrigger().whileTrue(new ElevatorSnap(m_elevator));
    m_secondary.pov(180).onTrue(m_elevator.snapDown());
    m_secondary.pov(0).onTrue(m_elevator.snapUp());
    //m_endeff.setDefaultCommand(new SequentialCommandGroup(
        //m_claw.pistonMove(false), m_arm.stop()));
    m_elevator.setDefaultCommand(m_elevManual);
    /* 
    // Auto Orient
    m_primaryJoystick.axisGreaterThan(6, .5).whileTrue(new AutoOrientCmd(drivebase, m_Limelight, 2, 18, 4.2, 2));

    // Auto Commands
    NamedCommands.registerCommand("pickup", m_pickup);
    NamedCommands.registerCommand("place left", new SequentialCommandGroup(m_elevator.setSnap(ElevatorLevel.POLE_TWO),
        new ParallelCommandGroup(new ElevatorSnap(m_elevator))));
    NamedCommands.registerCommand("grab", new autoClaw(m_endeff));
    NamedCommands.registerCommand("release", m_endeff.pistonMove(false));
    // Arm
    /*
     * m_endeff.setDefaultCommand(new EndManual(m_endeff));
     * m_secondary.rightTrigger().whileTrue(m_endeff.pistonMove(true));
     * m_secondary.rightTrigger().whileFalse(m_endeff.pistonMove(false));
     * m_secondary.a().whileTrue(new EndToAngle(m_endeff, 0.0));
     * m_secondary.b().whileTrue(new EndToAngle(m_endeff, 35.0));
     * m_secondary.y().whileTrue(new EndToAngle(m_endeff, 90.0));
     

    // Auto Orient
    m_primaryJoystick.axisGreaterThan(6, .5).whileTrue(new AutoOrientCmd(drivebase, m_Limelight, 2, 18, 4.2, 2));

    // Auto Commands
    NamedCommands.registerCommand("pickup", m_pickup);
    NamedCommands.registerCommand("place left", new SequentialCommandGroup(m_elevator.setSnap(ElevatorLevel.POLE_TWO),
        new ParallelCommandGroup(new ElevatorSnap(m_elevator))));
    NamedCommands.registerCommand("grab", new autoClaw(m_endeff));
    NamedCommands.registerCommand("release", m_endeff.pistonMove(false));
    // Arm
    /*
     * m_secondary.rightTrigger().whileTrue(m_endeff.pistonMove(true));
     * m_secondary.rightTrigger().whileFalse(m_endeff.pistonMove(false));
     * m_secondary.a().whileTrue(new EndToAngle(m_endeff, 0.0));
     * m_secondary.b().whileTrue(new EndToAngle(m_endeff, 35.0));
     * m_secondary.y().whileTrue(new EndToAngle(m_endeff, 90.0));
     *

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else {*/
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      xtraSlowTrig.onTrue(drivebase.driveFieldOriented(driveAngularVelocityXtraSlow));
      slowTrig.onTrue(drivebase.driveFieldOriented(driveAngularVelocitySlow));
      mediumTrig.onTrue(drivebase.driveFieldOriented(driveAngularVelocityMed));
      fastTrig.onTrue(drivebase.driveFieldOriented(driveAngularVelocityFast));
    
    /*
    if (Robot.isSimulation()) {
      primaryStartTrig.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_primaryJoystick.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      fullStopTrig.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driveSetDistanceTrig.whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      zeroGyroTrig.onTrue((Commands.runOnce(drivebase::zeroGyro)));
      primaryBackTrig.whileTrue(drivebase.centerModulesCommand());
      // backIsNegTrig.onTrue(Commands.none());
      // backIsPosTrig.onTrue(Commands.none());
    } else {
      zeroGyroTrig.onTrue((Commands.runOnce(drivebase::zeroGyro)));
      fakeVisionTrig.onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driveSetDistanceTrig.whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      primaryStartTrig.whileTrue(Commands.none());
      primaryBackTrig.whileTrue(Commands.none());
      // backIsNegTrig.whileTrue(Commands.runOnce(drivebase::lock,
      // drivebase).repeatedly());
      // backIsPosTrig.onTrue(Commands.none());

  }*/
    //zeroGyroTrig.onTrue((Commands.runOnce(drivebase::zeroGyro)));

    // COMMAND/TRIGGER ASSIGNMENTS

    //Primary Driver 
    deathModeTrig.whileTrue(drivebase.drive(DEATH_SPEEDS));
    fullStopTrig.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    //Secondary
    m_secondary.leftBumper()
        .onTrue(new SequentialCommandGroup(new EndToAngle(m_arm, 180.0), m_elevator.setSnap(ElevatorLevel.LOWEST), new ElevatorSnap(m_elevator)));
    // automatically bring elevator to 0 if left bumper pressed, first ensure
    // endeffector is in position
    m_elevUp.onTrue(m_elevator.snapUp());
    m_elevDown.onTrue(m_elevator.snapDown());
    m_elevGo.whileTrue(new ElevatorSnap(m_elevator));
    
    /* END TO ANGLE COMMANDS, UNTESTED
    m_secondary.a().onTrue(new EndToAngle(m_arm, 180.0));
    m_secondary.b().onTrue(new EndToAngle(m_arm, 215.0));
    m_secondary.x().onTrue(new EndToAngle(m_arm, 145.0));
    m_secondary.y().onTrue(new EndToAngle(m_arm, 225.0));
    */
    //m_arm.setDefaultCommand(new EndToAngle(m_arm, 180.0));
    m_arm.setDefaultCommand(m_arm.stop());
    spinIsOnTrig.whileTrue(m_armManual);
    
    m_coralRelease.whileTrue(m_claw.pistonMove(true));
    m_coralRelease.onFalse(m_claw.pistonMove(false));
    m_secondary.start().toggleOnTrue(new ToggleClaw(m_claw));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
