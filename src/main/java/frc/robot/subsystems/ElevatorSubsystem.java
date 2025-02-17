// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

public class ElevatorSubsystem extends SubsystemBase {
  public final SparkMax m_elevatorMotor1;
  public final SparkMax m_elevatorMotor2;
  public boolean isZeroed = false;

  public final SparkClosedLoopController m_elevatorFeedback;
  public DigitalInput elevatorLimit;
  private double currentTarget = 0;
  private RelativeEncoder elevatorEncoder;
  

  public ElevatorSubsystem() {
    m_elevatorMotor1 = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    m_elevatorMotor2 = new SparkMax(ElevatorConstants.lMotorID, MotorType.kBrushless);

    m_elevatorFeedback = m_elevatorMotor1.getClosedLoopController();
    elevatorEncoder = m_elevatorMotor1.getEncoder();

    elevatorLimit = new DigitalInput(0);
    
    m_elevatorMotor1.configure(Configs.ElevatorConfigs.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotor2.configure(Configs.ElevatorConfigs.elevatorConfig.follow(m_elevatorMotor1), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
 
  public void runElevatorMotorManual(double speed){
    m_elevatorMotor1.set(speed);
  }

  public void setPosition(double position){
    currentTarget = position;
  }

  public void zeroEncoder(){
    if(elevatorLimit.get() && !isZeroed){
      elevatorEncoder.setPosition(0);
      isZeroed = true;
    }
    else if(!elevatorLimit.get()){
      isZeroed = false;
    }
  }
  
    

    @Override
    
    public void periodic() {
      System.out.println(m_elevatorMotor1.getEncoder().getPosition());
      m_elevatorFeedback.setReference(currentTarget, ControlType.kMAXMotionPositionControl);
      zeroEncoder();
    }
  }


