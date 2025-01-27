package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.*;

import frc.robot.Constants.VisionConstants;

public class AlignCommand extends Command {
    private final VisionSubsystem m_Vision;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_Swerve;

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
   .withDeadband(4.73 * 0.1).withRotationalDeadband(2 * 0.1); // Add a 10% deadband


    public AlignCommand(VisionSubsystem vision, SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve, Distance holdDistance, int pipelineID) {
        m_Vision = vision;
        m_Swerve = swerve;

        addRequirements(m_Vision);
    }

    private double limelightAimProportional() {
        // kP (constant of proportionality)
        // Determines the aggressiveness of the proportional control loop
        double kP = 0.035;
    
        // Get the "tx" value from the Limelight
        double targetingAngularVelocity = m_Vision.getTX() * kP;
    
        // Convert to radians per second for the drivetrain
        targetingAngularVelocity *= 0.75;
    
        // Invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;
    
        return targetingAngularVelocity;
    }
    
    // Proportional ranging control with Limelight's "ty" value
    // Works best if the Limelight's mount height and target mount height are different.
    private double limelightRangeProportional() {
        double kP = 0.1;
    
        // Get the "ty" value from the Limelight
        double targetingForwardSpeed = m_Vision.getTY() * kP;
    
        // Convert to meters per second for the drivetrain
        targetingForwardSpeed *= TunerConstants.kSpeedAt12Volts.magnitude();
    
        // Invert the direction for proper control
        targetingForwardSpeed *= -1.0;
    
        return targetingForwardSpeed;
    }

    public void execute(){
        //double rot = limelightAimProportional();

        double xSpeed = limelightRangeProportional(); 

        m_Swerve.setControl(m_driveRequest.withVelocityX(xSpeed));
    }
}