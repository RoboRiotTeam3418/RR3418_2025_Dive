// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Setup;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorSnap extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_subsystem;
  public PIDController shooterController;
  public int height = 0;
  public int goalheight = 0; //in teirs
  public int levelstoTravel=0;
  public int direction=1;
  public double speed = Constants.getInstance().ElevatorSpeed;
  public double kP = Constants.getInstance().ElevatorP,kI = Constants.getInstance().ElevatorI,kD = Constants.getInstance().ElevatorD;
  public PIDController pid;

  /**
   * qCreates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorSnap(Elevator subsystem, double stopval) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(5, 10);//values suggested by wpilib documentation
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.higher){
      if (goalheight<=4){
        goalheight++;
      }else{
        goalheight = 0;
      }
      m_subsystem.higher=false;
    }
    if (goalheight!= height &&  Setup.getInstance().getSecondaryMoveElev()){
      levelstoTravel = goalheight-height;
      switch(goalheight) {
        case 0:
          // very small, home state
          break;
        case 1:
          // trough
          setpoint =
          break;
        case 2:
          // pole 1
          setpoint = 
          break;
        case 3:
          // pole 2
          setpoint = 
          break;
        case 4:
          // pole 3
          break;
        default:
          break;
  
      }
      m_subsystem.mot1.set(pid.calculate(m_subsystem.enc1.getPosition(), setpoint));
      m_subsystem.mot2.set(pid.calculate(m_subsystem.enc2.getPosition(), setpoint));
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pid.atSetpoint()){
      return true;
    }
    return false;
  }
}
