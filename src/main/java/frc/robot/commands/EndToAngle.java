// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.util.math.DeadbandUtils;

/** An example command that uses an example subsystem. */
public class EndToAngle extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CoralEndEffector m_subsystem;
  private double m_angle;
  // variables
  public SparkMax spinMotor;
  public AbsoluteEncoder spinEncoder;
  public DigitalInput gamePieceSensor;
  public Solenoid claw;
  public double allowance = 5;

  public double spinSpeed; // placeholder value
  public boolean isClockwise, isCounterClockwise;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param angle angle to go to in degrees
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
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("goal Angle", m_angle);  
    if (DeadbandUtils.isLess(m_subsystem.getEncValDegrees(), m_subsystem.POS_ANGLE_LIMIT)) {
      if (m_subsystem.getEncValDegrees() > m_angle + allowance) {
        spinMotor.set(-spinSpeed);
      }
      if (m_subsystem.getEncValDegrees() < m_angle - allowance) {
        spinMotor.set(spinSpeed);
      }
    }else{
      m_subsystem.stop();
    }
    if (DeadbandUtils.isWithin(m_subsystem.getEncValDegrees(),m_angle,allowance)){
      SmartDashboard.putBoolean("at Angle?", true);  
    }else{
      SmartDashboard.putBoolean("at Angle?", false);  
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
    return (DeadbandUtils.isWithin(spinEncoder.getPosition(), m_angle, allowance)
            ||DeadbandUtils.isGreater(m_subsystem.getEncValDegrees(), m_subsystem.POS_ANGLE_LIMIT));
  }
}
