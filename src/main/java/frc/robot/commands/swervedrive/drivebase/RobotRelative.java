package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class RobotRelative extends Command{
    
  private final SwerveSubsystem swerve;
  private final DoubleSupplier tX, tY;
  private final Double rot;

  public RobotRelative(SwerveSubsystem swerve, DoubleSupplier tX, DoubleSupplier tY, Double rot) {
    this.swerve = swerve;
    this.tX = tX;
    this.tY = tY;
    this.rot = rot;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  
  @Override
  public void execute()
  {
    swerve.drive(new Translation2d(tX.getAsDouble(),tY.getAsDouble()),rot,false);
  }
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
