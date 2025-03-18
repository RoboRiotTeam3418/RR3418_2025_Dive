// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.math.DeadbandUtils;

/** An example command that uses an example subsystem. */
public class EndToAngle extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Arm m_subsystem;
  private double m_angle;
  // variables
  public SparkMax spinMotor;
  public AbsoluteEncoder spinEncoder;
  public Solenoid claw;
  public double allowance = 10;

  public double setval; // placeholder value
  private PIDController pid;
  private final static double SPIN_P = .002, SPIN_I = 0.000000, SPIN_D = 0.00;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param angle angle to go to in degrees
   */
  public EndToAngle(Arm subsystem, Double angle) {
    m_subsystem = subsystem;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    spinMotor = m_subsystem.spinMotor;
    spinEncoder = m_subsystem.spinEncoder;
    pid = new PIDController(SPIN_P, SPIN_I, SPIN_D);
    pid.setTolerance(2, 5);// values suggested by wpilib documentation
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("goal Angle", m_angle);  
    //if (DeadbandUtils.isLess(m_subsystem.getEncValDegrees(), m_subsystem.POS_ANGLE_LIMIT)) {
      setval = pid.calculate(m_subsystem.spinEncoder.getPosition(), pid.getSetpoint());
      //if (m_subsystem.spinEncoder.getPosition()>)
      //SmartDashboard.putNumber("set Speed", setval);
      spinMotor.set(-setval);
    //}else{
     // m_subsystem.stop();
    //}
    if (DeadbandUtils.isWithin(m_subsystem.spinEncoder.getPosition(),m_angle,allowance)){
      SmartDashboard.putBoolean("at Angle?", true);  
      spinMotor.set(0);
    }else{
      SmartDashboard.putBoolean("at Angle?", false);  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //spinMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (DeadbandUtils.isWithin(m_subsystem.spinEncoder.getPosition(),m_angle,allowance));
    /* 
    return (DeadbandUtils.isWithin(spinEncoder.getPosition(), m_angle, allowance)
            ||DeadbandUtils.isGreater(m_subsystem.getEncValDegrees(), m_subsystem.POS_ANGLE_LIMIT));*/
  }
}
