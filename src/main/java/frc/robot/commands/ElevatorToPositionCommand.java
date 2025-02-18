

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.Meter;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.*;
import frc.robot.Constants.ElevatorConstants;


/** An example command that uses an example subsystem. */
public class ElevatorToPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_Elevator;

  private final TrapezoidProfile.Constraints constraints =  new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
  private final ProfiledPIDController m_ElevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,constraints);
  private final ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

  private final double setpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorToPositionCommand(ElevatorSubsystem elevator, double pos) {
    m_Elevator = elevator;
    setpoint = pos;
    
    m_ElevatorPID.setGoal(pos);
    m_ElevatorPID.setTolerance(ElevatorConstants.elevatorPosTolerance);
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double velocity = m_ElevatorPID.calculate(m_Elevator.getPosition())+m_ElevatorFeedforward.calculate(m_ElevatorPID.getSetpoint().velocity);
    m_Elevator.runElevatorMotorManual(velocity);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_ElevatorPID.atGoal();
  }
}