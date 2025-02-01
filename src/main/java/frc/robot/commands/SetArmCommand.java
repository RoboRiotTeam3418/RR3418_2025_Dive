package frc.robot.commands;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndEffector;

public class SetArmCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralEndEffector m_subsystem;
  private SparkMax motor;
  private AbsoluteEncoder encoder;
  private double speed;
  double angle;
  double allowance;
  /**
   * Creates a new setWristCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetArmCommand(CoralEndEffector subsystem, double targetAngle, double deadband) {
    m_subsystem = subsystem;
    motor=m_subsystem.spinMotor;
    encoder=m_subsystem.spinEncoder;
    speed=m_subsystem.spinSpeed;
    angle=targetAngle;
    allowance=deadband;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (encoder.getPosition()>angle+allowance) {
      motor.set(-speed);
    } else {
      motor.set(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (encoder.getPosition()>angle-allowance&&encoder.getPosition()<angle+allowance);
  }

}
