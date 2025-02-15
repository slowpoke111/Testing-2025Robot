// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.commands.ClawToPositionCommand;
import frc.robot.subsystems.ExampleSubsystem;
import static edu.wpi.first.units.Units.*;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClawSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Telemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ClawSubsystem m_claw = new ClawSubsystem();
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  double yOperator = m_operatorController.getLeftY();
  Trigger runIndexerTrigger = new Trigger(this::coralPresent);
  Trigger manualClawTrigger = new Trigger(() -> yOperator != 0);

  private final SendableChooser<Command> autoChooser;
  
  private final VisionSubsystem m_Vision = new VisionSubsystem();

  

  private final TimeOfFlight m_rangeSensor = new TimeOfFlight(ClawConstants.sensorID);

   public RobotContainer() {
      for (int port = 5800; port <= 5809; port++) {
          PortForwarder.add(port, "limelight.local", port);
      }

      NamedCommands.registerCommand("Align", new AlignCommand(m_drivetrain, m_Vision).withTimeout(2));
      
       m_rangeSensor.setRangingMode(RangingMode.Short, 24);
       autoChooser = AutoBuilder.buildAutoChooser("Tests");
       SmartDashboard.putData("Auto Mode", autoChooser);
 
       configureBindings();
   }
   public Command getAutonomousCommand() {
     // An example command will be run in autonomous
     return autoChooser.getSelected();
   }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));
        m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() ->
                drive.withVelocityX(m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_driverController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
        m_driverController.b().whileTrue(m_drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));
    
        m_driverController.x().whileTrue(new AlignCommand(m_drivetrain, m_Vision));


        m_driverController.pov(0).whileTrue(m_drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        m_driverController.pov(180).whileTrue(m_drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_driverController.leftBumper().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    double yDriver = m_driverController.getLeftY();

    m_operatorController.a().onTrue(new ClawToPositionCommand(m_claw, ClawConstants.L1ClawPosition));
    m_operatorController.b().onTrue(new ClawToPositionCommand(m_claw, ClawConstants.L2L3ClawPosition));
    m_operatorController.x().onTrue(new ClawToPositionCommand(m_claw, ClawConstants.L4ClawPosition));
    m_operatorController.y().onTrue(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPosition));

    // zero the claw angle . . . MAKE SURE TO DO THIS BEFORE DISABLING THE BOT OR GOING INTO A MATCH
    m_operatorController.rightBumper().onTrue(new ClawToPositionCommand(m_claw, 0));

    // failsafe for manual claw control
    manualClawTrigger.whileTrue(new InstantCommand(() -> m_claw.runClawMotor(yOperator * ClawConstants.manualClawSpeed)));

    //m_operatorController.rightTrigger().whileTrue(new InstantCommand(() -> m_claw.runShooterMotor(ClawConstants.fastShooterSpeed)));
    m_operatorController.leftTrigger().whileTrue(new RunShooterCommand(m_claw, ClawConstants.slowShooterSpeed));
    runIndexerTrigger.whileTrue(new RunShooterCommand(m_claw, ClawConstants.slowShooterSpeed));
  }

  public boolean coralPresent() {
    BooleanSupplier coralPresent = () -> (m_rangeSensor.getRange() > 1) && (m_rangeSensor.getRange() < 400);
    boolean isPresent = coralPresent.getAsBoolean();
    return isPresent;
  }


}