package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class intakeCommand extends Command {
    double value;
    CoralIntake intake;
    //boolean active;// what is this? took it out
    public intakeCommand(CoralIntake subsystem){
        this.intake=subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void execute(){
        intake.Intake();
    }
    @Override
    public boolean isFinished(){
        return intake.getCoralInHold();
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.Intake();
        }
    }
}
