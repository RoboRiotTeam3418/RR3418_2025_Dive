// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.drivers.Limelight;

/** An example command that uses an example subsystem. */
public class AutoOrientCmd extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveSubsystem m_Swerve; // Subsystem
  private final Limelight m_Limelight; // Limelight

  // Variables
  private int pipenum; // Pipeline to switch to
  private double desiredDistance; // How far you want to limelight to be away
  private double XTarget; // Target on X Axis / tx
  private double DB; // XTarget deadband
  private int chosenSide = 1; // Which limelight is active
  //private double Direction; // Left or right, depending on slider value listed in 'RobotContainer'

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoOrientCmd(SwerveSubsystem Veer, Limelight limejuice, int SelPipeline, double distance, double traget, double TargetDB) { // Sets everything up
    this.m_Swerve = Veer; // Drivetrain Subsystem
    this.m_Limelight = limejuice; // Limelight
    this.pipenum = SelPipeline;
    this.desiredDistance = distance;
    this.XTarget = traget;
    this.DB = TargetDB;
    //this.Direction = chosenDirection;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Veer);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limelight.SetPipeline(pipenum); // Chooses correct pipeline upon running.
    //this.chosenSide = m_Limelight.whichLimelightSees();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Start Orienting");

    m_Swerve.drive(new ChassisSpeeds(0, 0.0, 2));

    double limelightSees;
    double limelightArea;
    double limelightX;

    // ----- Print out important values ----- //
    /*if (chosenSide == 1) {
    */
      System.out.println("L1 chosen");
      limelightSees = m_Limelight.l1_tv.getDouble(0);
      limelightArea = m_Limelight.l1_ta.getDouble(0);
      limelightX = m_Limelight.l1_tx.getDouble(0);
      /* 
    }
    else if (chosenSide == -1) {
      System.out.println("L2 chosen");
      limelightSees = m_Limelight.l2_tv.getDouble(0);
      limelightArea = m_Limelight.l2_ta.getDouble(0);
      limelightX = m_Limelight.l2_tx.getDouble(0);
    }
    else {
      System.out.println("Side not chosen");
      return;
    }

    if (limelightSees != 1) {
      System.out.println("Limelight does not see target");
      return;
    }

    System.out.println("desiredDistance: " + this.desiredDistance);
    System.out.println("xTarget: " + this.XTarget);
    System.out.println("DB: " + this.DB);
    //System.out.println("Direction: " + this.Direction);
*/
    System.out.println(limelightArea);
    System.out.println(limelightX);
    System.out.println(limelightSees);
    
    // ----- Actual Code ----- //
    if (limelightArea < desiredDistance) { // Executes this code if limelight is far enough away from apriltag.
      System.out.println("Far away");
      if (limelightX > XTarget + DB) { // Executes this code if limelight is to the right and in bounds
        System.out.println("Rightdrive");
        m_Swerve.drive(new ChassisSpeeds(0.75, .6, 0.0)); 
      } else if (limelightX < XTarget - DB) { // executes this code if limelight is to the left and in bounds
        System.out.println("Leftdrive");
        m_Swerve.drive(new ChassisSpeeds(-0.75, .6, 0.0));
      } else { // Move forwards
        System.out.println("Forwarddrive");
        m_Swerve.drive(new ChassisSpeeds(0, .75, 0));
      }
    } else {
      System.out.println("Close");
      if (limelightX > XTarget + DB) { // Executes this code if limelight is to the right and inbounds
        System.out.println("CloseRight");
        m_Swerve.drive(new ChassisSpeeds(0.5, 0, 0.0));
      } else if (limelightX < XTarget - DB) { // executes this code if limelight is to the left and in bounds
        System.out.println("CloseLeft");
        m_Swerve.drive(new ChassisSpeeds((-0.5), 0, 0.0));
      }
    }

    System.out.println("End step Orienting");

    /*
      System.out.println("Orient Left");
      
      if (m_Limelight.l1_ta.getDouble(0) < desiredDistance) { // Executes this code if limelight is far enough away from apriltag.
      if (m_Limelight.l1_tx.getDouble(0) > XTarget + DB) { // Executes this code if limelight is to the right and in bounds
      m_Swerve.drive(new ChassisSpeeds((-0.4 * chosenSide), (0.16 * chosenSide), 0.0));
      } else if (m_Limelight.l1_tx.getDouble(0) < XTarget - DB) { // executes this code if limelight is to the left and in bounds
      m_Swerve.drive(new ChassisSpeeds((-0.4 * chosenSide), (-0.16 * chosenSide), 0.0));
      } else { // Move forwards
      m_Swerve.drive(new ChassisSpeeds(-0-.40, 0, 0));
      }
      }
    */
    // m_Swerve.drive(new ChassisSpeeds(1, 0, 0));

    SmartDashboard.putNumber("Limelight in Use: ", chosenSide);
    SmartDashboard.putBoolean("L1", m_Limelight.l1_tv.getDouble(00) == 1);
    SmartDashboard.putBoolean("L2", m_Limelight.l2_tv.getDouble(00) == 1);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(new ChassisSpeeds(0, 0, 0));
    System.out.println("Lost Target");
    System.out.println(chosenSide);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_Limelight.l1_tx.getDouble(0) <= XTarget + DB &&
    m_Limelight.l1_tx.getDouble(0) >= XTarget - DB &&
    m_Limelight.l1_ta.getDouble(0) >= desiredDistance /* L1 end */ || /* L2 begin*/ 
    m_Limelight.l2_tx.getDouble(0) <= XTarget + DB &&
    m_Limelight.l2_tx.getDouble(0) >= XTarget - DB &&
    m_Limelight.l2_ta.getDouble(0) >= desiredDistance /* L2 end */) ||
    (m_Limelight.l1_tv.getDouble(0) == 0 && m_Limelight.l2_tv.getDouble(0) == 0);

    //return (m_Limelight.l1_tv.getDouble(0) == 0 && m_Limelight.l2_tv.getDouble(0) == 0);
  }
}

/*
  Fun facts:The branches are 1 ft. + 1 in. (~33 cm) apart from eachother
  Complete apriltags are 10 + 1/2 in. long + wide.
  Apriltags on reef are 8.75 in. off the ground.

            m_Swerve.drive(new ChassisSpeeds(.5, .4, 0.0));
            front and back of robot = x
            left and right sides of robot = y




 */