// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClawToL1Command extends Command {
  private final ClawSubsystem m_claw;
  private final double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClawToL1Command(ClawSubsystem claw, double speed) {
    m_claw = claw;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    while(m_claw.getClawPosition() < ClawConstants.L1ClawPosition) {
      m_claw.runClawMotor(speed);
    }
    while(m_claw.getClawPosition() > ClawConstants.L1ClawPosition) {
      m_claw.runClawMotor(-speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.runClawMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_claw.getClawPosition() == ClawConstants.L1ClawPosition;
  }
}
