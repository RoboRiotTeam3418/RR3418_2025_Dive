package frc.robot.subsystems;

import frc.robot.Setup;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
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

    public double outtakeSpeed = -0.7, intakeSpeed = 0.6; //placeholder values
    public boolean intake, outtake;

    public CoralIntake(){
        intakeMotor = new SparkMax(Setup.INTAKE_END_ID, MotorType.kBrushed);
        gamePieceSensor = new DigitalInput(Setup.GamePieceSensorID);
    }

    public void Intake(){
        intakeMotor.set(intakeSpeed);
    }

    public void Outtake(){
        intakeMotor.set(outtakeSpeed);
    }

    public boolean getNoteInShooter(){
        return gamePieceSensor.get();
    }
  }