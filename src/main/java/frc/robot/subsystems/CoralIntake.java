package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Setup;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

    // get instance
    static CoralIntake mInstance = new CoralIntake();

    public static CoralIntake getInstance() {
        return mInstance;
    }

    //variables
    public SparkMax intakeMotor;
    public DigitalInput gamePieceSensor;
    public Solenoid pivot;

    public double outtakeSpeed = -0.7, intakeSpeed = Constants.getInstance().intakeSpeed; //placeholder value
    public boolean intake, outtake;

    public CoralIntake(){
        intakeMotor = new SparkMax(Setup.INTAKE_END_ID, MotorType.kBrushed);
        pivot= new Solenoid(0, null, 0);
    }

    public void Intake(){
        intakeMotor.set(intakeSpeed);
    }

    public void Outtake(){
        intakeMotor.set(outtakeSpeed);
    }
    public Command Pivot(boolean state){
        return run(
            ()-> {pivot.set(state);
        });
    }

    public boolean getCoralInHold(){
        return gamePieceSensor.get();
    }
  }