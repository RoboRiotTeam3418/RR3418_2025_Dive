// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Setup;
import frc.robot.subsystems.Elevator;
import frc.robot.util.math.DeadbandUtils;

/** An example command that uses an example subsystem. */
public class ElevatorManual extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Elevator m_subsystem;

  public final static double ELEVATOR_SPEED = 0.3;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorManual(Elevator subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elevator.getInstance().isManual = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DeadbandUtils.isGreater(Setup.getInstance().getSecondaryLY(), 0.1)) {
      m_subsystem.mot1.set(ELEVATOR_SPEED * Setup.getInstance().getSecondaryLY());
    } else {
      m_subsystem.mot1.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Elevator.getInstance().isManual) {
      return true;
    }
    return false;
  }
}
