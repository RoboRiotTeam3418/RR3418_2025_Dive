// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndEffector;

/** An example command that uses an example subsystem. */
public class autoClaw extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CoralEndEffector m_subsystem;
  private double m_angle;
  // variables
  public DigitalInput gamePieceSensor;
  public Solenoid claw;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoClaw(CoralEndEffector subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    claw = m_subsystem.claw;
    gamePieceSensor = m_subsystem.gamePieceSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.set(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.set(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gamePieceSensor.get();
  }
}
