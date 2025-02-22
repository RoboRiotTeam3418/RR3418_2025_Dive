// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* regular joystick controls climber like it did last year. However, we can switch this joystick control from climbing to
 (OtherSubsystem) and so on
 
 
 
 
 
 
 
 */ 

package frc.robot.subsystems;

import frc.robot.Setup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Setup;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
  //variables
  public SparkMax mot1,mot2;
  public AbsoluteEncoder enc1,enc2;

  public double climbSpeed = -0.2; //placeholder value | Is now the constant speed.
  public boolean armsDown;
  public Climber() {
    mot1 = new SparkMax(Setup.CLIMB1_ID, MotorType.kBrushless);
    enc1 = mot1.getAbsoluteEncoder();
    mot2 = new SparkMax(Setup.CLIMB2_ID, MotorType.kBrushless);
    enc2 = mot2.getAbsoluteEncoder();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ClimbSelf() { // Auto
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          while(enc1.getPosition() < Constants.getInstance().CLIMB_POS) {
            mot1.set(climbSpeed);
            mot2.set(-climbSpeed);
          }
        });
  }
  public Command ClimbMan(Boolean posOrNeg) { // Manual : Teleoperated
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {

        if (!Setup.isJoystickInDeadzone(Setup.getInstance().getSecondaryJoystick().getRightTriggerAxis())){
         if (Setup.isJoystickPositive(Setup.getInstance().getSecondaryJoystick().getRightTriggerAxis())) {
            while (enc1.getPosition() < Constants.getInstance().CLIMB_POS && enc2.getPosition() > -Constants.getInstance().CLIMB_POS) {
              this.mot1.set(climbSpeed);
              this.mot2.set(-climbSpeed);
            }
          } else {
            while (enc1.getPosition() < Constants.getInstance().CLIMB_POS && enc2.getPosition() > -Constants.getInstance().CLIMB_POS) {
              this.mot1.set(-climbSpeed);
              this.mot2.set(climbSpeed); 
            }
          }
        }

          /* if(posOrNeg == true && enc1.getPosition() < Constants.getInstance().CLIMB_POS) { // CLIMB_POS is a placeholder value
            while (enc1.getPosition() < Constants.getInstance().CLIMB_POS && enc2.getPosition() > -Constants.getInstance().CLIMB_POS) {
              this.mot1.set(climbSpeed);
              this.mot2.set(-climbSpeed);
            }
          } else if (posOrNeg == true && enc2.getPosition() < Constants.getInstance().CLIMB_POS) {
            while (enc1.getPosition() < Constants.getInstance().CLIMB_POS && enc2.getPosition() > -Constants.getInstance().CLIMB_POS) {
              this.mot1.set(-climbSpeed);
              this.mot2.set(climbSpeed); 
            }
        } */

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
  public void periodic() { // I don't think periodic will be helpful in this situation.
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() { // We don't use simulations
    // This method will be called once per scheduler run during simulation
  }
}
