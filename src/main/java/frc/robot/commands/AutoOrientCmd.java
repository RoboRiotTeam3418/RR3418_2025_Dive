// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.drivers.Limelight;

/** An example command that uses an example subsystem. */
public class AutoOrientCmd extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveSubsystem m_Swerve; // Subsystem + Limelight
  private final Limelight m_Limelight;

  private int pipenum; // Variables
  private double desiredDistance;
  private double XTarget;
  private double DB;
  private int chosenSide;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoOrientCmd(SwerveSubsystem Veer, Limelight limejuice, int SelPipeline, double distance, double traget,
      double TargetDB) { // Sets everything up
    this.m_Swerve = Veer; // Drivetrain Subsystem
    this.m_Limelight = limejuice; // Limelight
    this.pipenum = SelPipeline;
    this.desiredDistance = distance;
    this.XTarget = traget;
    this.DB = TargetDB;
    this.chosenSide = limejuice.whichLimelightSees();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Veer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limelight.SetPipeline(pipenum); // Chooses correct pipeline upon running.

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Orient Right");

    if (m_Limelight.l1_tv.getDouble(0) == 1 && chosenSide == 1) {
      if (m_Limelight.l1_ta.getDouble(0) < desiredDistance) { // Executes this code if limelight is far enough away from
        // apriltag.
        System.out.println("far enough");
        if (m_Limelight.l1_tx.getDouble(0) > XTarget + DB) { // Executes this code if limelight is to the right and in
          // bounds
          System.out.println("it drive pls");
          m_Swerve.drive(new ChassisSpeeds(.5, .4, 0.0));
        } else if (m_Limelight.l1_tx.getDouble(0) < XTarget - DB) { // executes this code if limelight is to the left
          // and
          // in bounds
          System.out.println("it drive pls other way");
          m_Swerve.drive(new ChassisSpeeds(-.5, .4, 0.0));
        } else { // Move forwards
          System.out.println("drive forward you flurmping robot");
          m_Swerve.drive(new ChassisSpeeds(00, .75, 0));
        }
      } else {
        if (m_Limelight.l1_tx.getDouble(0) > XTarget + DB) { // Executes this code if limelight is to the right and in
          // bounds
          System.out.println("it drive pls");
          m_Swerve.drive(new ChassisSpeeds(.25, 0, 0.0));
        } else if (m_Limelight.l1_tx.getDouble(0) < XTarget - DB) { // executes this code if limelight is to the left
          // and
          // in bounds
          System.out.println("it drive pls other way");
          m_Swerve.drive(new ChassisSpeeds((-.25), 0, 0.0));
        }
      }
    } else if (m_Limelight.l2_tv.getDouble(0) == 1 && chosenSide == -1) {
      if (m_Limelight.l2_ta.getDouble(0) < desiredDistance) { // Executes this code if limelight is far enough away from
        // apriltag.
        System.out.println("far enough");
        if (m_Limelight.l2_tx.getDouble(0) > XTarget + DB) { // Executes this code if limelight is to the right and in
          // bounds
          System.out.println("it drive pls");
          m_Swerve.drive(new ChassisSpeeds(.5, .4, 0.0));
        } else if (m_Limelight.l2_tx.getDouble(0) < XTarget - DB) { // executes this code if limelight is to the left
          // and in bounds
          System.out.println("it drive pls other way");
          m_Swerve.drive(new ChassisSpeeds(-.5, .4, 0.0));
        } else { // Move forwards
          System.out.println("drive forward you flurmping robot");
          m_Swerve.drive(new ChassisSpeeds(0, 0.75, 0));
        }

      } else {
        if (m_Limelight.l2_tx.getDouble(0) > XTarget + DB) { // Executes this code if limelight is to the right and in
          // bounds
          System.out.println("it drive pls");
          m_Swerve.drive(new ChassisSpeeds(.25, 0, 0.0));
        } else if (m_Limelight.l2_tx.getDouble(0) < XTarget - DB) { // executes this code if limelight is to the left
          // and
          // in bounds
          System.out.println("it drive pls other way");
          m_Swerve.drive(new ChassisSpeeds(-.25, 0, 0.0));
        }
      }
    } else {
      System.out.println("unseen");
    }

    /*
     * System.out.println("Orient Left");
     * 
     * if (m_Limelight.l1_ta.getDouble(0) < desiredDistance) { // Executes this code
     * if limelight is far enough away from
     * // apriltag.
     * if (m_Limelight.l1_tx.getDouble(0) > XTarget + DB) { // Executes this code if
     * limelight is to the right and in
     * // bounds
     * m_Swerve.drive(new ChassisSpeeds((-0.4 * chosenSide), (0.16 * chosenSide),
     * 0.0));
     * } else if (m_Limelight.l1_tx.getDouble(0) < XTarget - DB) { // executes this
     * code if limelight is to the left and
     * // in bounds
     * m_Swerve.drive(new ChassisSpeeds((-0.4 * chosenSide), (-0.16 * chosenSide),
     * 0.0));
     * } else { // Move forwards
     * m_Swerve.drive(new ChassisSpeeds(-0-.40, 0, 0));
     * }
     * }
     */
    // m_Swerve.drive(new ChassisSpeeds(1, 0, 0));
    SmartDashboard.putNumber("Limelight in Use", chosenSide);
    SmartDashboard.putBoolean("L1", m_Limelight.l1_tv.getDouble(00) == 1);
    SmartDashboard.putBoolean("L2", m_Limelight.l2_tv.getDouble(00) == 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(new ChassisSpeeds(0, 0, 0));

    System.out.println("Lost Target");
    System.out.println(m_Limelight.l1_tx.getDouble(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return (m_Limelight.l1_tx.getDouble(0) <= XTarget + DB&&
    // m_Limelight.l1_tx.getDouble(0) >= XTarget - DB&&
    // m_Limelight.l1_ta.getDouble(0) >= desiredDistance/* L1 end */|| /* L2 begin
    // */ m_Limelight.l2_tx.getDouble(0) <= XTarget + DB&&
    // m_Limelight.l2_tx.getDouble(0) >= XTarget - DB&&
    // m_Limelight.l2_ta.getDouble(0) >= desiredDistance /* L2 end */)||
    // (m_Limelight.l1_tv.getDouble(0) == 0 && m_Limelight.l2_tv.getDouble(0) == 0);

    // return (m_Limelight.l1_tv.getDouble(0) == 0 && m_Limelight.l2_tv.getDouble(0)
    // == 0);
  }
}

/*
 * Fun facts:
 * The branches are 1 ft. + 1 in. (~33 cm) apart from eachother
 * 
 * Complete apriltags are 10 + 1/2 in. long + wide.
 * 
 * Apriltags on reef are 8.75 in. off the ground.
 */