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
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import com.playingwithfusion.*;


import com.ctre.phoenix.motorcontrol.can.*;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. TalonFX motor will be final, SparkMax used for testing */
  //private final TalonFX m_elevatorMotor;
  private final SparkMax m_elevatorMotor;
  public boolean limitSwitch = false;  
  private final ElevatorFeedforward m_elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  private final PIDController m_elevatorFeedback = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  public DigitalInput elevatorLimit;
  public ElevatorSubsystem() {
    //m_elevatorMotor = new TalonFX(ElevatorConstants.kMotorID);
    m_elevatorMotor = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    m_elevatorFeedback.setTolerance(ElevatorConstants.kElevatorToleranceRPS);
}
 
  public void setElevator(double setpointRotationsPerSecond) {
    run(
      () -> {
        m_elevatorMotor.set(
          m_elevatorFeedforward.calculate(setpointRotationsPerSecond)
            + m_elevatorFeedback.calculate(
              m_elevatorMotor.get(), setpointRotationsPerSecond));
      });
    waitUntil(m_elevatorFeedback::atSetpoint).andThen(() -> m_elevatorMotor.set(1));
    if (limitSwitch == true){
      m_elevatorMotor.set(0);
    }
    }
  public boolean elevatorPosition(){
    if (elevatorLimit.get()){
      return true;
    }
    else{
      return false;
    }
  }
 

    @Override
    
    public void periodic() {
      // This method will be called once per scheduler run
      
    }
  }

