// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  private static double AUTO_DISTANCE = 6;
  private static double AUTO_SPEED = 0.4;

  public static Command exampleAuto(SwerveSubsystem drive) {
    return Commands.sequence(drive.driveToDistanceCommand(AUTO_DISTANCE, AUTO_SPEED) /* , placeholder command */);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
