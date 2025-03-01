// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public int goalheight = 0; //in tiers
  public boolean higher;
  public AnalogPotentiometer pot;
  public boolean isManual = true;
  public boolean toggle=false;

  

  public Elevator() {
    mot2 = new SparkMax(Setup.ELEVMOT1ID, MotorType.kBrushless);
    mot1 = new SparkMax(Setup.ELEVMOT2ID, MotorType.kBrushless);
    enc2 = mot2.getEncoder();
    enc1 = mot1.getEncoder();
    higher = Setup.getInstance().getSecondaryAasBool();
    pot =  new AnalogPotentiometer(1, 78, 0); //max height in inches is ~ 78
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command setSnap() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          if (goalheight<4){
            goalheight++;
          }else{
            goalheight = 0;
          }

        });
  }
  public double getElevPosition(){
    return pot.get();
  }
  public boolean getToggle() {
    return toggle;
  }
  public Command toggleMode() {
    return runOnce(()-> {
      toggle=!toggle;
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
    SmartDashboard.putNumber("snap height",goalheight);
    SmartDashboard.putBoolean("Mode", toggle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
