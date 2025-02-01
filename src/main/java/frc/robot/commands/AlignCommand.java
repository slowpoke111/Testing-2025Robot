package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.PIDControllerConfigurable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

public class AlignCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_Limelight;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05, 0, 0, 0.5);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.2);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.5);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
    this.m_drivetrain = drivetrain;
    this.m_Limelight = limelight;
    addRequirements(m_Limelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    RawFiducial fiducial;
    try {
      fiducial = m_Limelight.getFiducialWithId(10);

      final double rotationalRate = rotationalPidController.calculate(fiducial.txnc, 0) * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.5;
      final double velocityX = xPidController.calculate(fiducial.distToRobot, 1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.2;
      // final double velocityY = yPidController.calculate(fiducial.tync, 0) *
      // TunerConstants.MaxSpeed * 0.3;
        
      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint()) {
        this.end(true);
      }

      SmartDashboard.putNumber("txnc", fiducial.txnc);
      SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("xPidController", velocityX);
      m_drivetrain.setControl(
          alignRequest.withRotationalRate(-rotationalRate*1.5).withVelocityX(velocityX));
    } catch (VisionSubsystem.NoSuchTargetException nste) {
    }
  }

  @Override
  public boolean isFinished() {
    // return rotationalPidController.atSetpoint() && xPidController.atSetpoint()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
  }
}