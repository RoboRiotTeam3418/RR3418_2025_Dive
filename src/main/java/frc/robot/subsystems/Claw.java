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

public class Claw extends SubsystemBase {

  public DigitalInput gamePieceSensor;
  public Solenoid claw;
  public boolean ready = false;

  public Claw() {
    initialize();
  }

  private void initialize() {
    claw = new Solenoid(2,PneumaticsModuleType.REVPH, 0);
    gamePieceSensor = new DigitalInput(0);
  }

  public Command pistonMove(boolean state) {
    return runOnce(
        () -> {
          claw.set(state);
        });
  }

  public Boolean getClaw(){
    return claw.get();
  }

  public void updateReadiness(){
      ready = !ready;
  }

  public boolean getIsReady(){
    return ready;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("lazerboi", gamePieceSensor.get());  
    SmartDashboard.putBoolean("clawOpen", getClaw());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
