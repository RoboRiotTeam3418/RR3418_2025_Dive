package frc.robot.util.drivers;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.util.math.Rotation2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Setup;

public final class NavX extends Gyroscope {
    public final AHRS navX;

    static NavX mInstance;

    public static NavX getInstance() {
        if (mInstance == null)
            mInstance = new NavX(SerialPort.Port.kMXP);
    	return mInstance;
    }

    public NavX(edu.wpi.first.wpilibj.SerialPort.Port kmxp) {
        navX = new AHRS(NavXComType.kUSB2);
    }
/* 
    public NavX(edu.wpi.first.wpilibj.SerialPort.Port kmxp, byte updateRate) {
        navX = new AHRS(kmxp, AHRS.SerialDataType.kProcessedData, updateRate);
    }
*/
    public float getX(){

       return navX.getDisplacementX();

    }

    public float getY(){

        return navX.getDisplacementY();
 
     }

     public float getVelocityX(){
         return navX.getVelocityX();
     }

     public float getVelocityY(){
        return navX.getVelocityY();
    }

    public Rotation2d getRotation2d() {
        return navX.getRotation2d();
    }

    @Override
    public void calibrate() {
        navX.reset();
    }

    @Override
    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromRadians(getAxis(Axis.YAW));
    }

    @Override
    public double getUnadjustedRate() {
        return Math.toRadians(navX.getRate());
    }

    public double getAxis(Axis axis) {
        switch (axis) {
            case PITCH:
                return Math.toRadians(navX.getPitch());
            case ROLL:
                return Math.toRadians(navX.getRoll());
            case YAW:
                return Math.toRadians(navX.getYaw());
            default:
                return 0.0;
        }
    }

    public enum Axis {
        PITCH,
        ROLL,
        YAW
    }
}
