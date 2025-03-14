// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.thethriftybot.Conversion;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Setup;
import frc.robot.util.math.DeadbandUtils;

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
  public Double SPIN_OFFSET = 0.0, CONVERSION = 72.0, POS_ANGLE_LIMIT=34.9;
  //0ffset = distance from 0, Conversion= multiplier to get degrees, 
  //POS_ANGLE_LIMIT = 1/2 of range aka max angle from center counterclockwise (positive direction)

  public double spinSpeed = 0.01; // placeholder value
  public boolean isClockwise, isCounterClockwise;

  public CoralEndEffector() {
    initialize();
  }

  private void initialize() {
    spinMotor = new SparkMax(Setup.SPIN_ID, MotorType.kBrushless);
    spinEncoder = spinMotor.getAbsoluteEncoder();
    claw = new Solenoid(PneumaticsModuleType.REVPH, 0);
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
          if(DeadbandUtils.isLess(getEncValDegrees(), POS_ANGLE_LIMIT)){
            spinMotor.set(spinSpeed);
          }else{
            stop();
          }
        });
  }

  public Command spinCounterClockwise() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          if(DeadbandUtils.isLess(getEncValDegrees(), POS_ANGLE_LIMIT)){
            spinMotor.set(-spinSpeed);
          }else{
            stop();
          }
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
  public Boolean getClaw(){
    return claw.get();
  }

  public double getEncValDegrees(){
    return (spinEncoder.getPosition()+SPIN_OFFSET)*CONVERSION;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Raw Endeff Angle", spinEncoder.getPosition());
    SmartDashboard.putNumber("Adjusted Endeff Angle", getEncValDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
