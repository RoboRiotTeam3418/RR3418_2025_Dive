// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.util.drivers;

import java.lang.reflect.Array;

//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Changing to Shuffleboard.

public class Limelight{
  /** https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
   * 
   * tx = Horizontal offset of object relative to crosshair
   * ty = Vertical offset of object relative to crosshair. Doesn't matter as much as tx.
   * ta = Target area 0% to 100% of image. Basically, how much of the desired image it sees compared to how much of the background it sees.
   * tv = 1 if limelight sees a target it likes, 0 if not.
   * tl = Pipelines latency contribution (in ms). Unsure how latency is calculated.
   * 
  */


  // Actual Code
  NetworkTableInstance inst = NetworkTableInstance.getDefault(); // Default Networktable instance aka the first instance.
  NetworkTable table = inst.getTable("Limelight");

  public void SetPipeline(int Pipelinenum) { // void prevents function from sending back a value.
    table.getEntry("pipeline").setNumber(Pipelinenum); // Sets pipline number
  }


  public Limelight() { // NEW IDEA!!!: Just shove everything the limelight outputs into an array, then use that array to do stuff!
    int[] LimelightInfo = {2, 3};

    inst.startServer(); // Rev up those fryers! (Starts default server)
    inst.getTable("limelight").getEntry("<tx>").getDouble(0); // Refer to Line 45

    if (table.getEntry("tv").getInteger(0) == 1) { // If it see's a target it likes, it will do stuff
    
    // Time to learn swerve yay D;
    // Wait, maybe this can be used as a way to schedule commands instead? (mmmm maybe not, may execute too many commands at once. test later if possible.)
    }

  }



 /* PSUDOCODE + THOUGHT PROCESS
 * 
 * Assuming that limelight does indeed start it's own network table upon the server being started, we could just get the values we need from that table itself.
 * 
 *   inst.getTable("limelight").getEntry("<variablename>").getDouble(0);
 * 
 * 
 * 
 * 
 * 
 * 
 */



}
