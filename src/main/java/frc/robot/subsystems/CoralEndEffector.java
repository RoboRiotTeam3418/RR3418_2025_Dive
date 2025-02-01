// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< Updated upstream

public class CoralEndEffector extends SubsystemBase {
  public Solenoid piston;
  /** Creates a new ExampleSubsystem. */
  public CoralEndEffector() {
    piston = new Solenoid(0, null, 0);
  }
=======
import frc.robot.Setup;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

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
  public Solenoid piston;

  public double spinSpeed = 0.1; //placeholder value
  public boolean isClockwise, isCounterClockwise;

  /** Creates a new ExampleSubsystem. */
  public CoralEndEffector() {
    spinMotor = new SparkMax(Setup.getInstance().SPIN_ID, MotorType.kBrushless);
    piston = new Solenoid(0, null, 0);
    spinEncoder = spinMotor.getAbsoluteEncoder();
  }  
>>>>>>> Stashed changes

  
  /**
   * Example command factory method.
   *
   * @return a command
   */
<<<<<<< Updated upstream
  public Command exampleMethodCommand() {
=======
  public Command spin(int clockwise) {
>>>>>>> Stashed changes
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
<<<<<<< Updated upstream
          /* one-time action goes here */
        });
  }
=======
          spinMotor.set(Math.copySign(spinSpeed, clockwise));
        });
  }
  public Command to35() {
    if (spinEncoder.getPosition()>210){
      return run(
        () -> {
          while (spinEncoder.getPosition() >36){
            spinMotor.set(spinSpeed);
          }
        });
    }else{
      return run(
        () -> {
          while (spinEncoder.getPosition() <34){
            spinMotor.set(-spinSpeed);
          }
        });
    }
  }
>>>>>>> Stashed changes

  public Command to0() {
    if (spinEncoder.getPosition()>210){
      return run(
        () -> {
          while (spinEncoder.getPosition() >1){
            spinMotor.set(spinSpeed);
          }
        });
    }else{
      return run(
        () -> {
          while (spinEncoder.getPosition() <359){
            spinMotor.set(-spinSpeed);
          }
        });
    }
  }
  public Command stop() {
    return run(
      ()-> {
        spinMotor.set(0);
      });
  }

  public Command pistonMove(boolean state) {
    return runOnce(
      ()-> {
        piston.set(state);
      }
    );
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
