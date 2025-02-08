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



public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. TalonFX motor will be final, SparkMax used for testing */
  //private final TalonFX m_elevatorMotor;
  private final SparkMax m_elevatorMotor1;
  private final SparkMax m_elevatorMotor2;
  public boolean limitSwitch = false;
  public double x = 0;
  public final ElevatorFeedforward m_elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  public final PIDController m_elevatorFeedback = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  public DigitalInput elevatorLimit;
  public ElevatorSubsystem() {
    //m_elevatorMotor = new TalonFX(ElevatorConstants.kMotorID);
    m_elevatorMotor1 = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    m_elevatorMotor2 = new SparkMax(ElevatorConstants.lMotorID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(m_elevatorMotor1);
    m_elevatorMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorFeedback.setTolerance(ElevatorConstants.kElevatorToleranceRPS);
    setDefaultCommand(
      runOnce(
              () -> {
                m_elevatorMotor1.disable();
              })
          .andThen(run(() -> {}))
          .withName("Idle"));
  }
 
  public void setElevator(double setpointLocation) {
    m_elevatorFeedback.setSetpoint(setpointLocation);
    x= 
    m_elevatorFeedforward.calculate(ElevatorConstants.targetSpeed) //check
      + m_elevatorFeedback.calculate(
        m_elevatorMotor1.getEncoder().getPosition(), setpointLocation);
    m_elevatorMotor1.set(x);
    
    waitUntil(m_elevatorFeedback::atSetpoint).andThen(() -> m_elevatorMotor1.set(0));
    /*run(
      () -> {
        m_elevatorMotor1.set(
          m_elevatorFeedforward.calculate(ElevatorConstants.targetSpeed) //check
            + m_elevatorFeedback.calculate(
              m_elevatorMotor1.getEncoder().getPosition(), setpointLocation));
      });
    waitUntil(m_elevatorFeedback::atSetpoint).andThen(() -> m_elevatorMotor1.set(0));
    /*if (limitSwitch == true){
      m_elevatorMotor1.set(0);
    }
    */
    }
    
  public boolean elevatorLimitCheck(){
    return elevatorLimit.get();
  }
 

    @Override
    
    public void periodic() {
      // This method will be called once per scheduler run
      System.out.println("Test " + x);
      System.out.println(m_elevatorMotor1.getEncoder().getPosition());
    }
  }

