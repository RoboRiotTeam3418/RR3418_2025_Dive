// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* regular joystick controls climber like it did last year. However, we can switch this joystick control from climbing to
 (OtherSubsystem) and so on
 
 
 
 
 
 
 
 */

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Setup;

public class Climber extends SubsystemBase {

  private static final Double CLIMB_POS = 45.0;
  private static final double CLIMB_DEADBAND = 0.1;

  // variables
  public SparkMax mot1, mot2;
  public AbsoluteEncoder enc1, enc2;
  public Solenoid clamp;

  public double climbSpeed = -0.2; // placeholder value | Is now the constant speed.
  public boolean armsDown;

  public Climber() {
    initialize();
  }

  private void initialize() {
    //mot1 = new SparkMax(Setup.CLIMB1_ID, MotorType.kBrushless);
    //enc1 = mot1.getAbsoluteEncoder();
    //mot2 = new SparkMax(Setup.CLIMB2_ID, MotorType.kBrushless);
    //enc2 = mot2.getAbsoluteEncoder();
    //clamp = new Solenoid(0, null, 0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ClimbSelf() { // Auto
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          while (enc1.getPosition() < CLIMB_POS) {
            mot1.set(climbSpeed);
            mot2.set(climbSpeed);
          }
        });
  }

  @Override
  public void periodic() { // I don't think periodic will be helpful in this situation.
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() { // We don't use simulations
    // This method will be called once per scheduler run during simulation
  }
}
