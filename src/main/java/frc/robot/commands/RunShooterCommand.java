// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RunShooterCommand extends Command {
  private final ClawSubsystem m_claw;
  private final double speed;
  
 
  public RunShooterCommand(ClawSubsystem claw, double speed) {
    m_claw = claw;
    this.speed = speed;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
      m_claw.runShooterMotor(speed);
    }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_claw.runShooterMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}