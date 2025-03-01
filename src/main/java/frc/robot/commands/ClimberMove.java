// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Setup;
import frc.robot.subsystems.Climber;
import frc.robot.util.math.Deadbands;

/** An example command that uses an example subsystem. */
public class ClimberMove extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Climber m_subsystem;
  private final CommandXboxController m_secondary;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberMove(Climber subsystem, CommandXboxController secondary) {
    m_subsystem = subsystem;
    m_secondary = secondary;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // moves both climbers based on right joystick y value
    if (Setup.getInstance().getRightJoyIsOn()) {
      m_subsystem.mot1.set(m_subsystem.climbSpeed * Math.signum(m_secondary.getRightY()));
      m_subsystem.mot2.set(-m_subsystem.climbSpeed * Math.signum(m_secondary.getRightY()));
    } else {
      m_subsystem.mot1.set(0);
      m_subsystem.mot2.set(0);
    }
    // moves individual climbers up with the bumper and down with the trigger
    if (m_secondary.rightBumper().getAsBoolean()) {
      m_subsystem.mot1.set(m_subsystem.climbSpeed);
    }
    if (m_secondary.rightTrigger().getAsBoolean()) {
      m_subsystem.mot1.set(-m_subsystem.climbSpeed);
    }
    if (m_secondary.leftBumper().getAsBoolean()) {
      m_subsystem.mot2.set(-m_subsystem.climbSpeed);
    }
    if (m_secondary.leftTrigger().getAsBoolean()) {
      m_subsystem.mot2.set(-m_subsystem.climbSpeed);
    }

    if (Deadbands.isWithin(m_subsystem.enc1.getPosition(), m_subsystem.enc2.getPosition(), 5)
        && Deadbands.isWithin(m_subsystem.enc2.getPosition(), m_subsystem.enc1.getPosition(), 5)
        && m_subsystem.enc1.getPosition() >= 125) {
      m_subsystem.clamp.set(true);
    }
    if (Setup.getInstance().getSecondaryJoystick().x().getAsBoolean()) {
      m_subsystem.clamp.set(!m_subsystem.clamp.get());

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
