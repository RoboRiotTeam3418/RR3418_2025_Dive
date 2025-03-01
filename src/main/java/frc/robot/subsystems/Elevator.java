// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  public boolean higher, lower;
  private AnalogInput pot;
  public boolean isManual = true;
  public ElevatorLevel goalLevel = ElevatorLevel.LOWEST;

  public ShuffleboardTab tab = Shuffleboard.getTab("Driver");
  private GenericEntry goalheightEntry = tab.add("Goal Height Level", "").getEntry();

  public Dictionary<ElevatorLevel, Double> elevatorLeveltoHeightDictionary;// key is goal height in tiers, entry is
                                                                           // height to go to in inches

  public Elevator() {
    initialize();
  }

  private void initialize() {
    mot2 = new SparkMax(Setup.ELEVMOT1ID, MotorType.kBrushless);
    mot1 = new SparkMax(Setup.ELEVMOT2ID, MotorType.kBrushless);
    enc2 = mot2.getAlternateEncoder();
    enc1 = mot1.getAlternateEncoder();
    higher = Setup.getInstance().getSecondaryPOVUpasBool();
    lower = Setup.getInstance().getSecondaryPOVDownasBool();
    pot = new AnalogInput(3); // max height in inches is ~ 78

    initializeDictionary();
  }

  private void initializeDictionary() {
    elevatorLeveltoHeightDictionary = new Hashtable<>();
    // Adding key-value pairs
    elevatorLeveltoHeightDictionary.put(ElevatorLevel.LOWEST, 0.0); // very small, home state
    elevatorLeveltoHeightDictionary.put(ElevatorLevel.TROUGH, 21.0); // trough + 3in
    elevatorLeveltoHeightDictionary.put(ElevatorLevel.POLE_ONE, 32.0); // pole 1
    elevatorLeveltoHeightDictionary.put(ElevatorLevel.POLE_TWO, 48.0);// pole 2
    elevatorLeveltoHeightDictionary.put(ElevatorLevel.POLE_THREE, 75.0); // pole 3 + 3in
  }

  public static Elevator instance = new Elevator();

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
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
    return pot.getVoltage() * (75 / 5);
  }

  public double getHeightFromElevatorLevel(ElevatorLevel key) {
    return elevatorLeveltoHeightDictionary.get(key);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private ElevatorLevel incrementElevatorLevel(ElevatorLevel currentElevatorLevel) {

    ElevatorLevel[] values = ElevatorLevel.values();

    return values[(currentElevatorLevel.ordinal() + 1) % values.length];
  }

  private ElevatorLevel decrementElevatorLevel(ElevatorLevel currentElevatorLevel) {
    return ElevatorLevel.values()[(currentElevatorLevel.ordinal() - 1) % ElevatorLevel.values().length];
  }
  /*
   * updates height setting 
   * @param upOrDown if true going up, else going down
   */
  public Command setHeight(Boolean upOrDown){
    return runOnce(
        () -> {
          if (upOrDown) {
            incrementElevatorLevel(goalLevel);
            goalheightEntry.setDefaultString(goalLevel.toString());
          }else{
            decrementElevatorLevel(goalLevel);
            goalheightEntry.setDefaultString(goalLevel.toString());
          }
        });

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}