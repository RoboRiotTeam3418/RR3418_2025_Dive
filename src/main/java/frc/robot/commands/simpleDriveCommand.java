package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Setup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Utilities;
import frc.robot.util.drivers.SwerveModule;

public class simpleDriveCommand extends Command{
    private final Drivetrain drivetrain;
    private SwerveModule FLM;
    private SwerveModule FRM;
    private SwerveModule BLM;
    private SwerveModule BRM;
    double x;
    double y;
    double z;

    double speedChanger;
    ChassisSpeeds speeds;
    SwerveModuleState[] states=Setup.instance.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    public simpleDriveCommand(Drivetrain driver) {
    this.drivetrain=driver;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    FLM = drivetrain.frontLeftModule;
    FRM = drivetrain.frontRightModule;
    BLM = drivetrain.backLeftModule;
    BRM = drivetrain.backRightModule;
  }
  public void execute() {
    //deadbanding these values is VERY important, otherwise it'll never spin
        x = Utilities.deadband(Setup.getInstance().getPrimaryY());
        y = Utilities.deadband(Setup.getInstance().getPrimaryX());
        z = Utilities.deadband(Setup.getInstance().getPrimaryZ());
        speeds = new ChassisSpeeds(x,y,z);
        driveForAuto(speeds);
  }
  public void driveForAuto(ChassisSpeeds moveSpeeds) {
        speedChanger=.25;
        double rotation = moveSpeeds.omegaRadiansPerSecond *=-12 /
        Math.hypot(Setup.instance.WHEELBASE, Setup.instance.TRACKWIDTH);
        

        speeds = new ChassisSpeeds(-moveSpeeds.vxMetersPerSecond,moveSpeeds.vyMetersPerSecond, rotation);
        
        //swerve module states are a list of angles and speeds converted from the variables used in chassis. speeds below calculates them
        states = Setup.instance.kinematics.toSwerveModuleStates(speeds, new Translation2d(Setup.instance.TRACKWIDTH/2,0));


        if(x==0 && y==0 && z!=0){
                FLM.setTargetVelocity(z*speedChanger, -.25 * Math.PI);
                FRM.setTargetVelocity(-z*speedChanger, .25 * Math.PI);
                BLM.setTargetVelocity(z*speedChanger, .25 * Math.PI);
                BRM.setTargetVelocity(-z*speedChanger, -.25 * Math.PI);
                //System.out.println(z);
        } else if (x!=0 || y!=0 || z!=0){
                //System.out.println(states[0].speedMetersPerSecond);
                //System.out.println(speeds.omegaRadiansPerSecond);
                FLM.setTargetVelocity(states[0].speedMetersPerSecond*speedChanger, states[0].angle.getRadians());
                FRM.setTargetVelocity(states[1].speedMetersPerSecond*speedChanger, states[1].angle.getRadians());
                BLM.setTargetVelocity(states[2].speedMetersPerSecond*speedChanger, states[2].angle.getRadians());
                BRM.setTargetVelocity(states[3].speedMetersPerSecond*speedChanger, states[3].angle.getRadians());
        } else {
                FLM.setTargetVelocity(0, 0);
                FRM.setTargetVelocity(0, 0);
                BLM.setTargetVelocity(0, 0);
                BRM.setTargetVelocity(0, 0);
                //System.out.println("it's supposed to be STOPPED");
        }
}
  public boolean isFinished() {
    return false;
  }

}
