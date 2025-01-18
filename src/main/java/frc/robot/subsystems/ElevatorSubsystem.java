// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.playingwithfusion.*;


import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotorPort);
  
  private final Encoder m_elevatorEncoder =
    new Encoder(
      ElevatorConstants.kEncoderPorts[0],
      ElevatorConstants.kEncoderPorts[1],
      ElevatorConstants.kEncoderReversed);
    
  private final ElevatorFeedforward m_elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  private final PIDController m_elevatorFeedback = new PIDController(ElevatorConstants.elevatorP, 0.0, 0.0);
  public ElevatorSubsystem() {
    m_elevatorFeedback.setTolerance(ElevatorConstants.kElevatorToleranceRPS);
    m_elevatorEncoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
}

  public ElevatorSubsystem() {
    m_elevatorFeedback.setTolerance(ElevatorConstants.kElevatorToleranceRPS);
    m_elevatorEncoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
    

  }
  
 
  public void setElevator(double speed) {
    run(
      () -> {
        m_elevatorMotor.set(
          m_elevatorFeedforward.calculate(setpointRotationsPerSecond)
            + m_elevatorFeedback.calculate(
              m_elevatorEncoder.getRate(), setPointRotationsPerSecond));
      });
          
    }
  public double getHeight() {
    //Unsure how we will get the height in order to raise the elevator by the correct amount, so I have it here as an encoder
    return m_elevatorMotor.getEncoder().getPosition();
  }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  }

