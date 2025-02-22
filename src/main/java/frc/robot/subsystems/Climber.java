// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Setup;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
  //variables
  public SparkMax mot1,mot2;
  public AbsoluteEncoder enc1,enc2;
  public Solenoid clamp;

  public double climbSpeed = -0.2; //placeholder value
  public boolean armsDown;
  public Climber() {
    mot1 = new SparkMax(Setup.CLIMB1_ID, MotorType.kBrushless);
    enc1 = mot1.getAbsoluteEncoder();
    mot2 = new SparkMax(Setup.CLIMB2_ID, MotorType.kBrushless);
    enc2 = mot2.getAbsoluteEncoder();
    clamp = new Solenoid(0, null, 0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ClimbSelf() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          while(enc1.getPosition() < Constants.CLIMB_POS) {
            mot1.set(climbSpeed);
            mot2.set(-climbSpeed);
          }
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
