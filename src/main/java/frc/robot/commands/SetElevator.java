package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.playingwithfusion.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.*;

public class SetElevator extends Command {
    private final ElevatorSubsystem m_elevator;
    private final double m_setpoint;

    public SetElevator(ElevatorSubsystem elevator, double setpoint) {
        this.m_elevator = elevator;
        this.m_setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.m_elevatorFeedback.setSetpoint(m_setpoint);
    }

    @Override
    public void execute() {
        double output = m_elevator.m_elevatorFeedforward.calculate(ElevatorConstants.targetSpeed)
                        + m_elevator.m_elevatorFeedback.calculate(m_elevator.m_elevatorMotor1.getEncoder().getPosition(), m_setpoint);
        m_elevator.m_elevatorMotor1.set(output);
    }

    @Override
    public boolean isFinished() {
        return m_elevator.m_elevatorFeedback.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.m_elevatorMotor1.set(0);
    }
}
