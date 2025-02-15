package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MannualElevatorCommand extends Command{
    private final ElevatorSubsystem elevator;
    private final double speed; 

    public MannualElevatorCommand(ElevatorSubsystem elevatorSubsystem, double speed){
        elevator = elevatorSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize(){
        if(!elevator.getLimit()){
            elevator.runElevatorMotorManual(speed);
        }
    }

    @Override
    public void end(boolean interrupted){
        elevator.runElevatorMotorManual(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
