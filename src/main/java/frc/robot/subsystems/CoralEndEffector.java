// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Setup;

public class CoralEndEffector extends SubsystemBase {
  // get instance
  static CoralEndEffector mInstance = new CoralEndEffector();

  public static CoralEndEffector getInstance() {
    return mInstance;
  }

  // variables
  public SparkMax spinMotor;
  public AbsoluteEncoder spinEncoder;
  public DigitalInput gamePieceSensor;
  public Solenoid claw;

  public double spinSpeed = 0.1; // placeholder value
  public boolean isClockwise, isCounterClockwise;

  public CoralEndEffector() {
    initialize();
  }

  private void initialize() {
    //spinMotor = new SparkMax(Setup.SPIN_ID, MotorType.kBrushless);
    //spinEncoder = spinMotor.getAbsoluteEncoder();
    //claw = new Solenoid(PneumaticsModuleType.REVPH, 0);
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

  public Command pistonMove(boolean state) {
    return runOnce(
        () -> {
          claw.set(state);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          spinMotor.set(0);
        });
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
