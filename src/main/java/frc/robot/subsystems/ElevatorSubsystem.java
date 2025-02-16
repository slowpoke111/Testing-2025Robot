// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.io.ObjectInputFilter.Config.*;

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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;



public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. TalonFX motor will be final, SparkMax used for testing */
  //private final TalonFX m_elevatorMotor;
  public final SparkMax m_elevatorMotor1;
  public final SparkMax m_elevatorMotor2;
  public boolean limitSwitch = false;

  public final ElevatorFeedforward m_elevatorFeedforward = new ElevatorFeedforward(
    ElevatorConstants.kS, 
    ElevatorConstants.kG, 
    ElevatorConstants.kV, 
    ElevatorConstants.kA);
  public final SparkClosedLoopController m_elevatorFeedback;
  public DigitalInput elevatorLimit;
  private double currentTarget = 0.143;
  private RelativeEncoder elevatorEncoder;
  

  public ElevatorSubsystem() {
    m_elevatorMotor1 = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    m_elevatorMotor2 = new SparkMax(ElevatorConstants.lMotorID, MotorType.kBrushless);

    m_elevatorFeedback = m_elevatorMotor1.getClosedLoopController();
    elevatorEncoder = m_elevatorMotor1.getEncoder();
    
    m_elevatorMotor1.configure(Configs.ElevatorConfigs.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotor2.configure(Configs.ElevatorConfigs.elevatorConfig.follow(m_elevatorMotor1), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //m_elevatorFeedback.setTolerance(ElevatorConstants.kElevatorToleranceRPS);
  }
 
  public void runElevatorMotorManual(double speed){
    m_elevatorMotor1.set(speed);
  }

  public void setPosition(double position){
    currentTarget = position;
  }

  public boolean getLimit(){
    return elevatorLimit.get();
  }
  
    

    @Override
    
    public void periodic() {
      System.out.println(m_elevatorMotor1.getEncoder().getPosition());
      m_elevatorFeedback.setReference(currentTarget, ControlType.kMAXMotionPositionControl);
    }
  }


