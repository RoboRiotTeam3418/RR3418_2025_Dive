// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Setup;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ToggleClaw extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw m_subsystem;
  private boolean ready=false;
  private boolean buttondown = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ToggleClaw(Claw subsystem) {
    m_subsystem = subsystem;
    ready=false;
    buttondown=false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ready=false;
    buttondown=false;
    m_subsystem.claw.set(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(!buttondown&&!ready && Setup.getInstance().getSecondaryJoystick().back().getAsBoolean()){
      ready = true;
      buttondown=true;
    }else if (!buttondown&&ready&& Setup.getInstance().getSecondaryJoystick().back().getAsBoolean()){
      ready = false;
      buttondown=true;
    }
    if (!Setup.getInstance().getSecondaryJoystick().back().getAsBoolean()){
      buttondown=false;
    }
    if(ready && !m_subsystem.gamePieceSensor.get()){
      m_subsystem.claw.set(false);
      
    }
    if(!ready){
      m_subsystem.claw.set(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.claw.set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
