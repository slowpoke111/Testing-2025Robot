package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClawSubsystem extends SubsystemBase {
  private final TalonFX clawMotor = new TalonFX(ClawConstants.clawMotorID);
  private final TalonFX shooterMotor = new TalonFX(ClawConstants.shooterMotorID);
  private static final double fullRange = 360;
  private static final double expectedZero = 0;

  public ClawSubsystem() {
    shooterMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getClawPosition() {
    return MathUtil.clamp(
      (clawMotor.getPosition().getValueAsDouble() - (int) clawMotor.getPosition().getValueAsDouble()) * 360, expectedZero, fullRange);
  }
    
  public void runClawMotor(double speed) {
    clawMotor.set(speed);
  }

  public void runShooterMotor(double speed) {
    shooterMotor.set(speed);
  }
}
