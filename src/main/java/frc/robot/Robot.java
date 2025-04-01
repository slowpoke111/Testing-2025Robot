// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightHelpers;

import java.util.Timer;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.hardware.Pigeon2;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  Timer endOfMatchTimer = new Timer();

  private SwerveDrivePoseEstimator m_poseEstimator;

// Kinematics and Odometry
  private SwerveDriveKinematics m_kinematics;
  private SwerveModulePosition[] m_modulePositions;

  // Gyro (replace with actual gyro object if using a different one)
  private Pigeon2 gyro;

  // Vision Measurement Standard Deviations
  private static final double[] VISION_MEASUREMENT_STD_DEVS = {0.7, 0.7, 9999999};

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

@Override
public void robotInit() {
    gyro = new Pigeon2(25);

    // Define kinematics for swerve drive based on module locations
    m_kinematics = new SwerveDriveKinematics(
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY), // Front-left module
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY), // Front-right module
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY), // Back-left module
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY) // Back-right module
    );

    m_modulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(0.0, new Rotation2d()),
        new SwerveModulePosition(0.0, new Rotation2d()),
        new SwerveModulePosition(0.0, new Rotation2d()),
        new SwerveModulePosition(0.0, new Rotation2d())
    };

    // Initialize pose estimator
    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics,
        Rotation2d.fromDegrees(gyro.getAngle()), // Initial gyro reading
        m_modulePositions,
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, 0.01), // Odometry standard deviations
        VecBuilder.fill(0.7,0.7,9999999) // Vision standard deviations
    );

}


    // Sets the LED color to blue when the robot turns on
  

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
 @Override
public void robotPeriodic() {
  CommandScheduler.getInstance().run();
    // Update module positions periodically
    m_modulePositions = new SwerveModulePosition[] {m_robotContainer.m_drivetrain.getModule(0).getPosition(true),
      m_robotContainer.m_drivetrain.getModule(1).getPosition(true),
      m_robotContainer.m_drivetrain.getModule(2).getPosition(true),
      m_robotContainer.m_drivetrain.getModule(3).getPosition(true)};

    // Update pose estimator with odometry
    m_poseEstimator.update(
        Rotation2d.fromDegrees(gyro.getAngle()), 
        m_modulePositions
    );

    // Update vision measurement (if available)
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    boolean doRejectUpdate = false;

    // Ignore vision updates if gyro angular velocity is too high
    if (Math.abs(gyro.getRate()) > 360) {
        doRejectUpdate = true;
    }

    // Ignore vision updates if no valid tags detected
    if (mt2.tagCount == 0) {
        doRejectUpdate = true;
    }

    // Apply vision update if conditions are met
    if (!doRejectUpdate) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose, 
            mt2.timestampSeconds
        );
    }

    // Publish pose to SmartDashboard
    SmartDashboard.putString("Estimated Pose", m_poseEstimator.getEstimatedPosition().toString());
}


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    endOfMatchTimer.cancel();
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = new PathPlannerAuto(m_robotContainer.getAutonomousCommand()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

   // m_robotContainer.isTeleop = () -> false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

   // m_robotContainer.isTeleop = () -> true;

    // Sets the LED color to gold after match ends
    /* 
    endOfMatchTimer.schedule(new TimerTask() {
      @Override
      public void run() {
        m_led.LEDColor(LEDConstants.twinklesColorOneAndTwo);
      }
    }, new Date().getTime() + LEDConstants.matchTimeInMilliseconds);
    */
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
