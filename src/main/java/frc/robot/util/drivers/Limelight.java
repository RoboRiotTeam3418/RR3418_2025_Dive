// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drivers;

//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
  NetworkTableInstance inst = NetworkTableInstance.getDefault(); // Default Networktable instance aka the first
                                                                 // instance.

  NetworkTable LIMELIGHT1 = inst.getTable("limelight"); // Dr. Claw's Limelight
  public NetworkTableEntry l1_tx = LIMELIGHT1.getEntry("tx");
  public NetworkTableEntry l1_ty = LIMELIGHT1.getEntry("ty");
  public NetworkTableEntry l1_ta = LIMELIGHT1.getEntry("ta");
  public NetworkTableEntry l1_tv = LIMELIGHT1.getEntry("tv");

  NetworkTable LIMELIGHT2 = inst.getTable("limelight-limeone"); // E.D.D's Limelight
  public NetworkTableEntry l2_tx = LIMELIGHT2.getEntry("tx");
  public NetworkTableEntry l2_ty = LIMELIGHT2.getEntry("ty");
  public NetworkTableEntry l2_ta = LIMELIGHT2.getEntry("ta");
  public NetworkTableEntry l2_tv = LIMELIGHT2.getEntry("tv");

  public void SetPipeline(int Pipelinenum) { // void prevents function from sending back a value.
    LIMELIGHT1.getEntry("pipeline").setNumber(Pipelinenum); // Sets pipline number
    LIMELIGHT2.getEntry("pipeline").setNumber(Pipelinenum);
  }

  public boolean comparetemp() {
    return (l1_tv.getDouble(0.0) == 1);
  }

  public int whichLimelightSees() {
    if (l1_tv.getDouble(0) == 1) { // Unnamed Limelight Sees
      return 1;
    } else if (l2_tv.getDouble(0) == 1) { // Limeone sees
      return -1;
    } else if (l1_tv.getDouble(0) == 1 && l2_tv.getDouble(0) == 1) { // Both limelights see

      if ((l1_ta.getDouble(0) - l2_ta.getDouble(0)) >= 0) { // Closer apriltag takes prioity
        return 1;
      } else {
        return -1;
      }

    } else {
      return 0; // No limelight sees
    }
  }

  Double[] LimelightInfoArray = new Double[4]; // Creates new 'double' array with 4 elements

  public Limelight() {
    inst.startServer(); // Rev up those fryers! (Starts default server)

    LimelightInfoArray[0] = l1_tx.getDouble(0.0);
    LimelightInfoArray[1] = l1_ty.getDouble(0.0);
    LimelightInfoArray[2] = l1_ta.getDouble(0.0);
    LimelightInfoArray[3] = l1_tv.getDouble(0.0);
  }

  /*
   * public void outputToSmartDashboard() {
   * SmartDashboard.putNumber("Limelight:tx", tx.getDouble(0.0));
   * SmartDashboard.putNumber("Limelight:ty", ty.getDouble(0.0));
   * SmartDashboard.putNumber("Limelight:ta", ta.getDouble(0.0));
   * SmartDashboard.putNumber("Limelight:tv", tv.getDouble(0.0));
   * }
   */

  /**
   * https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
   * 
   * tx = Horizontal offset of object relative to crosshair
   * ty = Vertical offset of object relative to crosshair. Doesn't matter as much
   * as tx.
   * ta = Target area 0% to 100% of image. Basically, how much of the desired
   * image it sees compared to how much of the background it sees.
   * tv = 1 if limelight sees a target it likes, 0 if not.
   * tl = Pipelines latency contribution (in ms). Unsure how latency is
   * calculated.
   * 
   */

}
