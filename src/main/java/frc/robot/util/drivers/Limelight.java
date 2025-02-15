// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.util.drivers;

import java.lang.reflect.Array;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
//import edu.wpi.first.networktables.Topic;
//import edu.wpi.first.networktables.DoubleTopic;

//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Changing to Shuffleboard.
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight{
  NetworkTableInstance inst = NetworkTableInstance.getDefault(); // Default Networktable instance aka the first instance.
  NetworkTable table = inst.getTable("limelight");
  public NetworkTableEntry tx = table.getEntry("tx");
  public NetworkTableEntry ty = table.getEntry("ty");
  public NetworkTableEntry ta = table.getEntry("ta");
  public NetworkTableEntry tv = table.getEntry("tv");


  public void SetPipeline(int Pipelinenum) { // void prevents function from sending back a value.
    table.getEntry("pipeline").setNumber(Pipelinenum); // Sets pipline number
  }

  public boolean comparetemp() {
    return (tv.getDouble(0.0) == 1);
  }

  public double TXDistance(){
    return 0.0;
  }




  Double[] LimelightInfoArray = new Double[4]; // Creates new 'double' array with 4 elements

  public Limelight() { 
    inst.startServer(); // Rev up those fryers! (Starts default server)

    LimelightInfoArray[0] = tx.getDouble(0.0);
    LimelightInfoArray[1] = ty.getDouble(0.0);
    LimelightInfoArray[2] = ta.getDouble(0.0);
    LimelightInfoArray[3] = tv.getDouble(0.0);
  }

  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("Limelight:tx", tx.getDouble(0.0));
    SmartDashboard.putNumber("Limelight:ty", ty.getDouble(0.0));
    SmartDashboard.putNumber("Limelight:ta", ta.getDouble(0.0));
    SmartDashboard.putNumber("Limelight:tv", tv.getDouble(0.0));
  }

    /** https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
   * 
   * tx = Horizontal offset of object relative to crosshair
   * ty = Vertical offset of object relative to crosshair. Doesn't matter as much as tx.
   * ta = Target area 0% to 100% of image. Basically, how much of the desired image it sees compared to how much of the background it sees.
   * tv = 1 if limelight sees a target it likes, 0 if not.
   * tl = Pipelines latency contribution (in ms). Unsure how latency is calculated.
   * 
  */

}

