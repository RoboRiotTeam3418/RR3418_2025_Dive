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
  
  //private final Elevator m_elevator = new Elevator();
  /*private final CoralEndEffector m_endeff = new CoralEndEffector();
  private final Climber m_climber = new Climber();
  private final Example_Subsystem m_exampleSubsystem = new Example_Subsystem();
  //private final SwerveSubsystem m_drivetrain = new SwerveSubsystem();

  //commands
  private final Command m_climb = new ClimberMove(m_climber);*/
  //private final Command m_snap = new ElevatorSnap(m_elevator);
  //private final Command m_manual = new ElevatorManual(m_elevator);
  //private final Command m_simpDrive = new simpleDriveCommand(m_drivetrain);



  // Replace with CommandPS4Controller or CommandJoystick if needed

  CommandJoystick m_primaryJoystick = Setup.getInstance().getPrimaryJoystick();
  //CommandXboxController m_secondary = Setup.getInstance().getSecondaryJoystick();
  public double speed = 0;
  //Driver speeds
  /* 
  public Double getSpeedSetting(){
  public String speedSetting = "medium";
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
      }*/
  public Double getXSpeedSetting(){
  //set the speed based on the current speed setting
     double sign = 1;
      //String whichSpeed = speedSetting;
      if(Setup.getInstance().getDeathMode()){
              speed =Constants.MAX_SPEED;
      } else if(Setup.getInstance().getPrimaryDriverXButton()){
              speed=-0.325;
      } else if(Setup.getInstance().getPrimaryDriverAButton()){
              speed=-0.5;
      } else if(Setup.getInstance().getPrimaryDriverBButton()){
              speed=-0.825;
      } else if(Setup.getInstance().getPrimaryDriverYButton()){
              speed = -.999;
      }
      if (m_primaryJoystick.getX()>0.1){
        sign = -1;
      }else if(m_primaryJoystick.getX()<0.1){
        sign = 1;
      }
      return speed*sign;
  }
  public Double getYSpeedSetting(){
    //set the speed based on the current speed setting
       double sign = 1;
        //String whichSpeed = speedSetting;
        if(Setup.getInstance().getDeathMode()){
                speed =Constants.MAX_SPEED;
        } else if(Setup.getInstance().getPrimaryDriverXButton()){
                speed=-0.325;
        } else if(Setup.getInstance().getPrimaryDriverAButton()){
                speed=-0.5;
        } else if(Setup.getInstance().getPrimaryDriverBButton()){
                speed=-0.825;
        } else if(Setup.getInstance().getPrimaryDriverYButton()){
                speed = -.999;
        }
        if (m_primaryJoystick.getY()>0.1){
          sign = 1;
        }else if(m_primaryJoystick.getY()<0.1){
          sign = -1;
        }
        return speed*sign;
    }
  

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
    // limit acceleration
    SlewRateLimiter xfilter = new SlewRateLimiter(0.25,-0.9,0);
    SlewRateLimiter yfilter = new SlewRateLimiter(0.25, -0.9,0);
   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  DoubleSupplier rotSupplier = () -> drivebase.getRot(m_primaryJoystick.getTwist());
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> yfilter.calculate(m_primaryJoystick.getY()) + getXSpeedSetting(),// CHECK FUNCTION
                                                                () -> xfilter.calculate(m_primaryJoystick.getX()) + getYSpeedSetting())// CHECK FUNCTION
                                                            .withControllerRotationAxis(rotSupplier)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  public DoubleSupplier getNegTwist = ()-> m_primaryJoystick.getTwist()*-1;
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_primaryJoystick::getTwist, getNegTwist)//checkfunction
                                                           .headingWhile(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocity.copy()
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


    Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
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

    //m_secondary.leftBumper().whileTrue(m_endeff.spinCounterClockwise());
    //m_secondary.rightBumper().whileTrue(m_endeff.spinClockwise());
    //m_secondary.b().onTrue(m_endeff.to35());

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

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
