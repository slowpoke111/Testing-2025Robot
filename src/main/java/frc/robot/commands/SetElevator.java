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
import com.revrobotics.spark.SparkBase.ControlType;
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
    private final double m_setpoint;
    private final ElevatorSubsystem elevator;

    public SetElevator(ElevatorSubsystem elevator, double position) {
        this.elevator = elevator;
        m_setpoint = position;
    }

    @Override
    public void initialize() {
        System.out.println("Button Pressed");
        elevator.setPosition(m_setpoint);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
