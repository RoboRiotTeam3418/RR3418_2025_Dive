// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Setup;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorSnap extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_subsystem;
  private boolean m_inAuto, m_override;
  public PIDController shooterController;
  //public int levelstoTravel=0;
  //public int direction=1;
  public double setval, m_setheight;
  public double kP = Constants.getInstance().ElevatorP,kI = Constants.getInstance().ElevatorI,kD = Constants.getInstance().ElevatorD;
  public PIDController pid;
   public AnalogPotentiometer pot;
  public double allowance = 2; //inches 

  /**
   * 
   *
   * @param subsystem The subsystem used by this command.
   * @param Override True if we don't need to push the button to make the elevator move (ie autonomous or one button start position)
   * @param setheight only matters if override is true, else please put -1 for clarity
   */
  public ElevatorSnap(Elevator subsystem, boolean Override, double setheight) {
    m_subsystem = subsystem;
    m_setheight =setheight;
    m_override = Override;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elevator.getInstance().isManual = false;
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(5, 10);//values suggested by wpilib documentation
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    //if the goal is different from the acceptable range and is allowed to go because either a button was pushed or it was overridden for auto
    if (((m_subsystem.goalToDistance(m_subsystem.goalheight) < m_subsystem.getElevPosition() - allowance)|| (m_subsystem.goalToDistance(m_subsystem.goalheight) > m_subsystem.getElevPosition() + allowance)) &&  (Setup.getInstance().getSecondaryMoveElev()||m_override)){

      //ACCOUNT FOR CHASSIS HEIGHT LATER
      switch(m_subsystem.goalheight) {
        case 0:
          // very small, home state
          pid.setSetpoint(m_subsystem.goalToDistance(0));
          break;
        case 1:
          // trough + 3in
          pid.setSetpoint(m_subsystem.goalToDistance(1));
          break;
        case 2:
          // pole 1
          pid.setSetpoint(m_subsystem.goalToDistance(2));
          break;
        case 3:
          // pole 2
          pid.setSetpoint(m_subsystem.goalToDistance(3));
          break;
        case 4:
          // pole 3 + 3in
          pid.setSetpoint(m_subsystem.goalToDistance(4));
          break;
        default:
          break;
  
      }
      if(m_override){
        pid.setSetpoint(m_setheight);
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
    if (pid.atSetpoint()){
      Elevator.getInstance().isManual = true;
    }
    return pid.atSetpoint();
  }
}
