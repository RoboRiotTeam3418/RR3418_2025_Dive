// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.subsystems.Example_Subsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.drivers.Limelight;

//import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoOrientCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_Swerve; // Subsystem + Limelight
  private final Limelight m_Limelight;

  private int pipenum; // Variables
  private double desiredDistance;
  private double XTarget;
  private double DB;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoOrientCmd(SwerveSubsystem Veer, Limelight limejuice, int SelPipeline, double distance, double traget, double TargetDB) {
    this.m_Swerve = Veer; // Drivetrain Subsystem
    this.m_Limelight = limejuice; // Limelight
    this.pipenum = SelPipeline;
    this.desiredDistance = distance;
    this.XTarget = traget;
    this.DB = TargetDB;

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
 
   if (m_Limelight.ta.getDouble(0) < desiredDistance) {
   if (m_Limelight.tx.getDouble(0) > XTarget + DB) {
    m_Swerve.drive(new ChassisSpeeds(-0.4,0.16,0.0)); // oRPS: 0.5
   } else if (m_Limelight.tx.getDouble(0) < XTarget - DB) {
    m_Swerve.drive(new ChassisSpeeds(-0.4,0.-.16,0.0)); // oRPS: -0.5
   } else {
    m_Swerve.drive(new ChassisSpeeds(-0.40,0,0));
   }
   }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(new ChassisSpeeds(0,0,0));

    System.out.println("Lost Target");
    System.out.println(m_Limelight.tx.getDouble(0));

   /*if (! interrupted) {
      System.out.println("target in vision");
    } */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return Math.abs(m_Limelight.tx.getDouble(0)) < 1;
  }
}
