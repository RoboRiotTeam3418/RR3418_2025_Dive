// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Setup;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkMax mot2;
  public SparkMax mot1;
  public RelativeEncoder enc2;
  public RelativeEncoder enc1;
  public PIDController elevController;
  public boolean higher;
  public AnalogPotentiometer pot;
  private RobotConfig config;
  

  public Elevator() {
    mot2 = new SparkMax(Setup.ELEVMOT1ID, MotorType.kBrushless);
    mot1 = new SparkMax(Setup.ELEVMOT2ID, MotorType.kBrushless);
    enc2 = mot2.getEncoder();
    enc1 = mot1.getEncoder();
    higher = Setup.getInstance().getSecondaryAasBool();
    pot =  new AnalogPotentiometer(0, 78, 0); //max height in inches is ~ 78


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public double getElevPosition(){
    return pot.get();
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
