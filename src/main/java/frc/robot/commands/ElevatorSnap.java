// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.util.math.DeadbandUtils;

/** An example command that uses an example subsystem. */
public class ElevatorSnap extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Elevator m_subsystem;
  private double setval;
  private PIDController pid;
  private final static double ALLOWANCE = 1; // inches

  private final static double ELEVATOR_P = .05, ELEVATOR_I = 0.0025, ELEVATOR_D = 0.00;

  /**
   * 
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorSnap(Elevator subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);
    pid.setTolerance(2, 5);// values suggested by wpilib documentation
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if the goal is different from the acceptable range and is allowed to go
    // because either a button was pushed or it was overridden for auto
    // if (((m_subsystem.goalToDistance(m_subsystem.goalheight) <
    // m_subsystem.getElevPosition() - allowance)||
    // (m_subsystem.goalToDistance(m_subsystem.goalheight) >
    // m_subsystem.getElevPosition() + allowance)) &&
    // (Setup.getInstance().getSecondaryMoveElev()||m_override)){
    if (DeadbandUtils.isOutside(m_subsystem.getHeightFromElevatorLevel(m_subsystem.goalLevel),
        m_subsystem.getElevPosition(), ALLOWANCE)) {
      // ACCOUNT FOR CHASSIS HEIGHT LATER
      pid.setSetpoint(m_subsystem.getHeightFromElevatorLevel(m_subsystem.goalLevel));
      setval = pid.calculate(m_subsystem.getElevPosition(), pid.getSetpoint());
      m_subsystem.mot1.set(-setval);
      m_subsystem.mot2.set(-setval);
      SmartDashboard.putNumber("targetSpeed", -setval);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.mot1.set(0);
    m_subsystem.mot2.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
