// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final VisionSubsystem m_Vision;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      inst.setServerTeam(5181);
      inst.startDSClient();

      var limelightNT = inst.getTable("limelight");
      DoubleTopic txTopic = limelightNT.getDoubleTopic("tx");
      DoubleTopic tyTopic = limelightNT.getDoubleTopic("ty");

      DoubleSubscriber txSubscriber = txTopic.subscribe(-30.0);
      DoubleSubscriber tySubscriber = tyTopic.subscribe(-30.0);

      DoubleSupplier txSupplier = txSubscriber::get;
      DoubleSupplier tySupplier = tySubscriber::get;

      m_Vision = new VisionSubsystem(txSupplier, tySupplier);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition);
        // .onTrue(new ExampleCommand(m_exampleSubsystem));
    //m_driverController.a().onTrue(new PrintCommand(NetworkTableInstance.getDefault().getTopic("limelight").getProperties()));
    m_driverController.b().onTrue(new PrintCommand(Double.toString(m_Vision.getTY())));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
  public void robotInit() 
  {
      // Make sure you only configure port forwarding once in your robot code.
      // Do not place these function calls in any periodic functions
      for (int port = 5800; port <= 5809; port++) {
          PortForwarder.add(port, "limelight.local", port);
      }
  }
}
