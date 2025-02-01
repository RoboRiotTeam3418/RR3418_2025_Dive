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
  public PIDController shooterController;
  public int height = 0;//nothing changes height?
  public int goalheight = 0; //in tiers
  //public int levelstoTravel=0;
  //public int direction=1;
  public double setval;
  public double speed = Constants.getInstance().ElevatorSpeed;
  public double kP = Constants.getInstance().ElevatorP,kI = Constants.getInstance().ElevatorI,kD = Constants.getInstance().ElevatorD;
  public PIDController pid;
  private int target;
   public AnalogPotentiometer pot;
   

  /**
   * qCreates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorSnap(Elevator subsystem, int targetPoint) {
    m_subsystem = subsystem;
    target=targetPoint;
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
    //if (goalheight!= height &&  Setup.getInstance().getSecondaryMoveElev()){
      //levelstoTravel = goalheight-height;
      switch(target) {
        case 0:
          // very small, home state
          pid.setSetpoint(1);
          break;
        case 1:
          // trough
          pid.setSetpoint(21);
          break;
        case 2:
          // pole 1
          pid.setSetpoint(32);
          break;
        case 3:
          // pole 2
          pid.setSetpoint(48);
          break;
        case 4:
          // pole 3
          pid.setSetpoint(75);
          break;
        default:
          break;
        //could default be the 0 value?
      }
      setval = pid.calculate(pot.get(), pid.getSetpoint());
      m_subsystem.mot1.set(setval);
      m_subsystem.mot2.set(setval);
    
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
