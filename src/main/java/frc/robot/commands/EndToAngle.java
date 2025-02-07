// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Setup;
import frc.robot.subsystems.CoralEndEffector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class EndToAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralEndEffector m_subsystem;
  private double m_angle;
    //variables
  public SparkMax spinMotor;
  public AbsoluteEncoder spinEncoder;
  public DigitalInput gamePieceSensor;
  public Solenoid claw;

  public double spinSpeed; //placeholder value
  public boolean isClockwise, isCounterClockwise;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EndToAngle(CoralEndEffector subsystem, Double angle) {
    m_subsystem = subsystem;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    spinSpeed = m_subsystem.spinSpeed;
    addRequirements(subsystem);
    spinMotor = m_subsystem.spinMotor;
    spinEncoder = m_subsystem.spinEncoder;
    claw = m_subsystem.claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (spinEncoder.getPosition()> (m_angle+180)%360){
      while (spinEncoder.getPosition() > m_angle+1){
        spinMotor.set(spinSpeed);}
    }else{
      while (spinEncoder.getPosition() <m_angle-1){
       spinMotor.set(-spinSpeed);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return spinEncoder.getPosition() > m_angle - 1 && spinEncoder.getPosition() < m_angle + 1;
  }
}
