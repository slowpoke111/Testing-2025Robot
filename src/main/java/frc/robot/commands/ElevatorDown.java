// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;

/** An example command that uses an example subsystem. */
public class ElevatorDown extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

      private final ElevatorSubsystem m_elevator;
  
      /**
       * Creates a new ElevatorUp command.
       *
       * @param targetHeight The target height for the elevator.
       * @param elevator The elevator subsystem used by this command.
       */
      public ElevatorDown(DoubleSupplier targetHeight, ElevatorSubsystem elevator) {
          super(
              new PIDController(ElevatorConstants.elevatorP, ElevatorConstants.elevatorI, ElevatorConstants.elevatorD),
              elevator::getHeight,
              targetHeight::getAsDouble,
              elevator::setElevator,
              elevator
          );
          m_elevator = elevator;
          
          getController().setTolerance(ElevatorConstants.elevatorPosTolerance, ElevatorConstants.elevatorVelTolerance);
        
          // Use addRequirements() here to declare subsystem dependencies.
          addRequirements(elevator);
      }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
