// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Setup;
import frc.robot.subsystems.Arm;
import frc.robot.util.math.DeadbandUtils;
import frc.robot.util.drivers.Toggles;

/** An example command that uses an example subsystem. */
public class ArmManual extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Arm m_subsystem;

  public final double SPIN_SPEED;
  public static final double MAX_VAL = 230,NEG_MAX_VAL = 180-(MAX_VAL-180), SLOW_VAL = 215, NEG_SLOW_VAL=180-(SLOW_VAL-180);
  private double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmManual(Arm subsystem) {
    m_subsystem = subsystem;
    SPIN_SPEED = m_subsystem.spinSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  public double getRX(){
    return Setup.getInstance().getSecondaryRX();

  }
  /*public boolean inSlowZoneLeft(){
    return m_subsystem.spinEncoder.getPosition()()< 360-SLOW_VAL && (m_subsystem.spinEncoder.getPosition()() >180);
  }
  public boolean inSlowZoneRight(){
    return m_subsystem.spinEncoder.getPosition()()> SLOW_VAL && (m_subsystem.spinEncoder.getPosition()() <180);
  }
  public boolean inDeadZoneLeft(){
    return (m_subsystem.spinEncoder.getPosition()()< 360-MAX_VAL) && (m_subsystem.spinEncoder.getPosition()() >180);
  }
  public boolean inDeadZoneRight(){
    return (m_subsystem.spinEncoder.getPosition()()> MAX_VAL) && (m_subsystem.spinEncoder.getPosition()()< 180);
  }*/

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set arm speed and direction based on controller
    if(DeadbandUtils.isGreater(Setup.getInstance().getSecondaryRX(), .1)) {
      speed = SPIN_SPEED*Setup.getInstance().getSecondaryRX();
    } /*else {
      speed = 0;
    }*/
    //if the arm is in the slow zone or dead zone and going the wrong way,
    //scale down the speed 

    if (( m_subsystem.spinEncoder.getPosition()>= SLOW_VAL && getRX()>0.1) || ( m_subsystem.spinEncoder.getPosition()<= NEG_SLOW_VAL && getRX()<-0.1)) {
      speed/=10;
    }
    if (( m_subsystem.spinEncoder.getPosition()>= MAX_VAL && getRX()>0.1) || ( m_subsystem.spinEncoder.getPosition()<= NEG_MAX_VAL && getRX()<-0.1)) {
      speed=0;
    }
    m_subsystem.spinMotor.set(-speed);
    SmartDashboard.putNumber("spin speed", speed);
    
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
