// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorLevel;
import frc.robot.Setup;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkMax mot2;
  public SparkMax mot1;
  public RelativeEncoder enc2;
  public RelativeEncoder enc1;
  public PIDController elevController;
  public boolean higher, lower, toggle = true;
  private SparkAnalogSensor pot;
  public ElevatorLevel goalLevel = ElevatorLevel.LOWEST;

  public Dictionary<ElevatorLevel, Double> elevatorLeveltoHeightDictionary;// key is goal height in tiers, entry is
                                                                           // height to go to in inches

  public Elevator() {
    initialize();
  }

  private void initialize() {
    mot2 = new SparkMax(Setup.ELEVMOT2ID, MotorType.kBrushless);
    mot1 = new SparkMax(Setup.ELEVMOT1ID, MotorType.kBrushless);
    enc2 = mot2.getAlternateEncoder();
    enc1 = mot1.getAlternateEncoder();
    // higher = Setup.getInstance().getSecondaryPOVUpasBool();
    // lower = Setup.getInstance().getSecondaryPOVDownasBool();
    pot = mot2.getAnalog(); // max height in inches is ~ 78

    initializeDictionary();
  }

  private void initializeDictionary() {
    elevatorLeveltoHeightDictionary = new Hashtable<>();
    // Adding key-value pairs
    elevatorLeveltoHeightDictionary.put(ElevatorLevel.LOWEST, 1.0); // very small, home state
//    elevatorLeveltoHeightDictionary.put(ElevatorLevel.TROUGH, 13.0); // trough + 3in
    elevatorLeveltoHeightDictionary.put(ElevatorLevel.POLE_ONE, 7.0); // pole 1
    elevatorLeveltoHeightDictionary.put(ElevatorLevel.POLE_TWO, 24.0);// pole 2
    elevatorLeveltoHeightDictionary.put(ElevatorLevel.POLE_THREE, 48.0); // pole 3 + 3in
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command stop() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          mot1.set(0);
          mot2.set(0);
        });
  }

  public double getElevPosition() {
    return (pot.getVoltage() / 1.55) * 50 - 2;
  }

  public double getHeightFromElevatorLevel(ElevatorLevel key) {
    return elevatorLeveltoHeightDictionary.get(key);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("snap height", goalLevel.ordinal());
    SmartDashboard.putNumber("Current Height", getElevPosition());
  }

  public Command setSnap(ElevatorLevel key) {
    return runOnce(
        () -> {
          goalLevel = key;
        });
  }

  public Command snapDown() {
    return runOnce(
        () -> {
          decrementElevatorLevel(goalLevel);
        });
  }

  public Command snapUp() {
    return runOnce(
        () -> {
          incrementElevatorLevel(goalLevel);
        });
  }

  private void incrementElevatorLevel(ElevatorLevel currentElevatorLevel) {
    ElevatorLevel[] values = ElevatorLevel.values();

    int newIndex = currentElevatorLevel.ordinal() + 1;
    goalLevel = values[newIndex % values.length];
  }

  private void decrementElevatorLevel(ElevatorLevel currentElevatorLevel) {
    ElevatorLevel[] values = ElevatorLevel.values();
    int newIndex = currentElevatorLevel.ordinal()-1;
    if (newIndex<0){
      newIndex = values.length-1;
    }
    goalLevel = values[newIndex];
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}