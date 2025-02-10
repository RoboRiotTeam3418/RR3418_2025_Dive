// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Setup;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimberMove extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberMove(Climber subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Setup.getInstance().getRightJoyIsPos()) {
      m_subsystem.mot1.set(m_subsystem.climbSpeed);
      m_subsystem.mot2.set(-m_subsystem.climbSpeed);
    }else if (Setup.getInstance().getRightJoyIsNeg()){
      m_subsystem.mot1.set(-m_subsystem.climbSpeed);
      m_subsystem.mot2.set(m_subsystem.climbSpeed);
    }else{
      m_subsystem.mot1.set(0);
      m_subsystem.mot2.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
