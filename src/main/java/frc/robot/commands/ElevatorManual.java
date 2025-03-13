// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Setup;
import frc.robot.subsystems.Elevator;
import frc.robot.util.math.DeadbandUtils;
import frc.robot.util.drivers.Toggles;

/** An example command that uses an example subsystem. */
public class ElevatorManual extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Elevator m_subsystem;

  public final static double ELEVATOR_SPEED = .75;
  private double speed;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DeadbandUtils.isGreater(Setup.getInstance().getSecondaryLY(), .1)) {
      speed = ELEVATOR_SPEED*Setup.getInstance().getSecondaryLY();
    } else {
      speed = 0;
    };
    if ((m_subsystem.getElevPosition()>35&&Setup.getInstance().getSecondaryLY()<0)||(m_subsystem.getElevPosition()<13&&Setup.getInstance().getSecondaryLY()>0)) {
      speed/=2;
    }
    if ((m_subsystem.getElevPosition()>45&&Setup.getInstance().getSecondaryLY()<0)||(m_subsystem.getElevPosition()<5&&Setup.getInstance().getSecondaryLY()>0)) {
      speed=0;
    }
    m_subsystem.mot1.set(speed);
    m_subsystem.mot2.set(speed);
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
