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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Setup;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkMax mot2;
  public SparkMax mot1;
  public PIDController elevController;
  public boolean higher,lower;
  public AnalogInput pot;
  public boolean isManual = true;
  public int goalheight = 0; //in teirs
  public ShuffleboardTab tab = Shuffleboard.getTab("Driver");
  private GenericEntry goalheightEntry =
      tab.add("Goal Height Level", 0)
         .getEntry();
  public Dictionary<Integer, Double> goalToDist;// key is goal height in tiers, entry is height to go to in inches
  

  public Elevator() {
    mot2 = new SparkMax(Setup.ELEVMOT1ID, MotorType.kBrushless);
    mot1 = new SparkMax(Setup.ELEVMOT2ID, MotorType.kBrushless);
    enc2 = mot2.getAlternateEncoder();
    enc1 = mot1.getAlternateEncoder();
    higher = Setup.getInstance().getSecondaryPOVUpasBool();
    lower = Setup.getInstance().getSecondaryPOVDownasBool();
    pot =  new AnalogInput(3); //max height in inches is ~ 78
    
    goalToDist = new Hashtable<>();
        // Adding key-value pairs
        goalToDist.put(0, 0.0); // very small, home state
        goalToDist.put(1, 21.0); // trough + 3in
        goalToDist.put(2, 32.0); // pole 1
        goalToDist.put(3, 48.0);// pole 2
        goalToDist.put(4, 75.0); // pole 3 + 3in

    
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
  public double getElevPosition(){
    return pot.getVoltage()*(75/5);
  }
  public double goalToDistance(Integer key){
    return goalToDist.get(key);
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
    if (higher){
      if (goalheight<4){
        goalheight++;
      }else{
        goalheight = 0;
      }
      goalheightEntry.setDouble(goalheight);
      higher=false;
    }
    if (lower){
      if (goalheight>0){
        goalheight--;
      }else{
        goalheight = 4;
      }
      goalheightEntry.setDouble(goalheight);
      higher=false;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
