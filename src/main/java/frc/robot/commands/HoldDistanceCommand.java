package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class HoldDistanceCommand extends Command {
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_swerve;
    private final VisionSubsystem m_Vision;
    private final SwerveRequest.FieldCentric m_driveRequest;
    private final ProfiledPIDController m_pidController;
    private final Distance targetDistance;

    // PID constants
    private static final double MAX_VELOCITY = 1.0; 
    private static final double MAX_ACCELERATION = 0.5; 

    private static final double P = 0.1;
    private static final double I = 0.01;
    private static final double D = 0.05;

    public HoldDistanceCommand(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve, VisionSubsystem vision ,Distance targetDistance) {
        this.m_swerve = swerve;
        this.m_Vision = vision;
        this.targetDistance = targetDistance;

        m_pidController = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        m_pidController.setGoal(targetDistance.in(Meters));

        m_driveRequest = new SwerveRequest.FieldCentric().withDeadband(0.1); 

        addRequirements(m_Vision);
    }

    @Override
    public void initialize() {
        m_pidController.reset(m_Vision.getDistance(0, VisionConstants.CORAL_APRILTAG_HEIGHT.in(Inches)).in(Meters));
    }

    @Override
    public void execute() {
        double currentDistance = m_Vision.getDistance(0, VisionConstants.CORAL_APRILTAG_HEIGHT.in(Inches)).in(Meters);
        double velocityOutput = m_pidController.calculate(currentDistance);

        // Only allow forward movement if the robot is farther than the target
        if (currentDistance < targetDistance.in(Meters)) {
            m_swerve.setControl(m_driveRequest.withVelocityX(velocityOutput).withVelocityY(0).withRotationalRate(0));
        } else {
            // Stop movement when closer to the target
            m_swerve.setControl(m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.setControl(m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return m_pidController.atGoal();
    }
}
