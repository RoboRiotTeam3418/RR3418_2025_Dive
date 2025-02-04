// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Setup;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralEndEffector extends SubsystemBase {
  // get instance
  static CoralEndEffector mInstance = new CoralEndEffector();

  public static CoralEndEffector getInstance() {
      return mInstance;
  }
  //variables
  public SparkMax spinMotor;
  public AbsoluteEncoder spinEncoder;
  public DigitalInput gamePieceSensor;

  public double spinSpeed = 0.1; //placeholder value
  public boolean isClockwise, isCounterClockwise;

  public CoralEndEffector() {
    spinMotor = new SparkMax(Setup.getInstance().SPIN_ID, MotorType.kBrushless);
    spinEncoder = spinMotor.getAbsoluteEncoder();
  }  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command spinClockwise() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          spinMotor.set(spinSpeed);
        });
  }
  public Command spinCounterClockwise() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          spinMotor.set(-spinSpeed);
        });
  }
  public Command toAngle(Double angle) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    if (spinEncoder.getPosition()> (angle+180)%360){
      return run(
        () -> {
          while (spinEncoder.getPosition() > angle+1){
            spinMotor.set(spinSpeed);
          }
        });
    }else{
      return run(
        () -> {
          while (spinEncoder.getPosition() <angle-1){
            spinMotor.set(-spinSpeed);
          }
        });
    }
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
