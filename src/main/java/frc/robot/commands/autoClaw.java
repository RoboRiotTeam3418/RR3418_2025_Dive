// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Setup;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.util.math.DeadbandUtils;

/** An example command that uses an example subsystem. */
public class EndManual extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CoralEndEffector m_subsystem;
  private double m_angle;
  // variables
  public SparkMax spinMotor;
  public AbsoluteEncoder spinEncoder;
  public DigitalInput gamePieceSensor;
  public Solenoid claw;

  public double spinSpeed; // placeholder value
  public boolean isClockwise, isCounterClockwise;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EndManual(CoralEndEffector subsystem) {
    m_subsystem = subsystem;
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
    if (DeadbandUtils.isGreater(Setup.getInstance().getSecondaryRX(), .1)) {
      spinSpeed = Setup.getInstance().getSecondaryRX();
    } else {
      spinSpeed = 0;
    }
    /*
     * if ((spinEncoder.getPosition()>35&&Setup.getInstance().getSecondaryLY()<0)||(
     * spinEncoder.getPosition()<13&&Setup.getInstance().getSecondaryLY()>0)) {
     * spinSpeed/=2;
     * }
     * if ((spinEncoder.getPosition()>45&&Setup.getInstance().getSecondaryLY()<0)||(
     * spinEncoder.getPosition()<5&&Setup.getInstance().getSecondaryLY()>0)) {
     * spinSpeed=0;
     * }
     */
    spinMotor.set(spinSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
