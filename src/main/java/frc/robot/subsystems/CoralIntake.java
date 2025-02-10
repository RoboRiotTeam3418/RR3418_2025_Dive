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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Solenoid;

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
    public Solenoid pivot;

    public CoralIntake(){
        intakeMotor = new SparkMax(Setup.INTAKE_END_ID, MotorType.kBrushed);
        gamePieceSensor = new DigitalInput(Setup.GamePieceSensorID);
        pivot = new Solenoid(1, null, 1);
    }

    public Command Pivot(Boolean goingUp){
        return runOnce(
            () -> {
                pivot.set(goingUp);
            });
    }
    public Command Intake(){
        return run(
            () -> {
                intakeMotor.set(intakeSpeed);
            });
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