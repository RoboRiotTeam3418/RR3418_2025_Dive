// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Setup;
import frc.robot.subsystems.Elevator;
import frc.robot.util.math.DeadbandUtils;

/** An example command that uses an example subsystem. */
public class ElevatorSnap extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Elevator m_subsystem;
  private final CommandXboxController m_secondary;
  private boolean m_inAuto, m_override;
  private PIDController shooterController;
  private double setval, m_setheight;
  private PIDController pid;
  private AnalogPotentiometer pot;
  private final static double ALLOWANCE = 2; // inches

  private final static double ELEVATOR_P = 0, ELEVATOR_I = 0, ELEVATOR_D = 0;

  /**
   * 
   *
   * @param subsystem The subsystem used by this command.
   * @param override  True if we don't need to push the button to make the
   *                  elevator move (ie autonomous or one button start position)
   * @param setHeight only matters if override is true, else please put -1 for
   *                  clarity
   */
  public ElevatorSnap(Elevator subsystem, boolean override, double setHeight) {
    m_subsystem = subsystem;
    m_setheight = setHeight;
    m_override = override;
    m_secondary = Setup.getInstance().getSecondaryJoystick();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elevator.getInstance().isManual = false;
    pid = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);
    pid.setTolerance(5, 10);// values suggested by wpilib documentation
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
        m_subsystem.getElevPosition(), ALLOWANCE) && (Setup.getInstance().getSecondaryMoveElev() || m_override)) {
      // ACCOUNT FOR CHASSIS HEIGHT LATER
      pid.setSetpoint(m_subsystem.getHeightFromElevatorLevel(m_subsystem.goalLevel));

      if (m_override || m_secondary.rightBumper().getAsBoolean()) {
        pid.setSetpoint(m_setheight);
        m_override = true;
      } else {
        pid.setSetpoint(m_subsystem.getElevPosition());
      }
      setval = pid.calculate(pot.get(), pid.getSetpoint());
      m_subsystem.mot1.set(setval);
      m_subsystem.mot2.set(setval);
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
    if (pid.atSetpoint()) {
      Elevator.getInstance().isManual = true;
    }
    return pid.atSetpoint();
  }
}
