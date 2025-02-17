// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Radian;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class ClawToPositionCommand extends Command {
  private final ClawSubsystem m_claw;
  private PIDController clawPID = new PIDController(ClawConstants.kP, 0, ClawConstants.kD);
  private Angle desiredPosition;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClawToPositionCommand(ClawSubsystem claw, Angle desiredPosition) {
    m_claw = claw;
    this.desiredPosition = desiredPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      clawPID.setTolerance(ClawConstants.tolerance);
      clawPID.setSetpoint(desiredPosition.in(Radian));
      clawPID.enableContinuousInput(0, Math.PI * ClawConstants.GEAR_RATIO);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MathUtil.clamp(clawPID.calculate(m_claw.getClawPosition().in(Radian), desiredPosition.in(Radian)), -0.5, 0.5);

   /* if (speed < 0 && desiredPosition.in(Radian) < m_claw.getClawPosition().in(Radian)) {
      m_claw.runClawMotor(0.5 * speed);
    }
    else if (speed > 0 && desiredPosition.in(Radian) > m_claw.getClawPosition().in(Radian)) {
      m_claw.runClawMotor(0.5 * speed);
    }
    else if (speed < 0 && desiredPosition.in(Radian) > m_claw.getClawPosition().in(Radian)) {
      m_claw.runClawMotor(0.5 * speed);
    }
    else if (speed > 0 && desiredPosition.in(Radian) < m_claw.getClawPosition().in(Radian)) {
      m_claw.runClawMotor(0.5 * speed);
    } */

    m_claw.runClawMotor(0.5 * speed);

    if (clawPID.atSetpoint()) {
      m_claw.runClawMotor(0);
    }
    else {
      m_claw.runClawMotor(MathUtil.clamp(clawPID.calculate(m_claw.getClawPosition().in(Radian), desiredPosition.in(Radian)), -0.5, 0.5));
    }

    SmartDashboard.putNumber("Angle", m_claw.getClawPosition().in(Radian));
    SmartDashboard.putNumber("PID Speed", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.runClawMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return clawPID.atSetpoint();
  }
}
