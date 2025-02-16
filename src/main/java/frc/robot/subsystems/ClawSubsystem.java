package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClawSubsystem extends SubsystemBase {
  private final TalonFX clawMotor = new TalonFX(ClawConstants.clawMotorID);
  private final TalonFX shooterMotor = new TalonFX(ClawConstants.shooterMotorID);
  private static final double expectedZero = 0;

  public ClawSubsystem() {
    shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    clawMotor.setNeutralMode(NeutralModeValue.Brake);
    clawMotor.setPosition(Angle.ofBaseUnits(expectedZero, Radian));
  }

  public double getClawPosition() {
    return clawMotor.getPosition().getValue().in(Radian);
  }
    
  public void runClawMotor(double speed) {
    clawMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle", getClawPosition());
    System.out.println(getClawPosition());
  }

  public void runShooterMotor(double speed) {
    shooterMotor.set(speed);
  }
}
