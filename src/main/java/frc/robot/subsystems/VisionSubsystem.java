package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.*;

public class VisionSubsystem extends SubsystemBase {
  private RawFiducial[] fiducials;

  public VisionSubsystem() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {

    SmartDashboard.putNumber("Rotational P", 0.3);
    SmartDashboard.putNumber("Move P", 0.3);
    SmartDashboard.putNumber("Rotational I", 0.3);
    SmartDashboard.putNumber("Move I", 0.3);
    SmartDashboard.putNumber("Rotational D", 0.3);
    SmartDashboard.putNumber("Move D", 0.3);

    // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
    LimelightHelpers.setCameraPose_RobotSpace(
        "", //LL name
        0.3556, 
        0.1016,
        0.3429,
        0,
        -2,
        0);
    LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] {1,5,8,9,10,11,12});
  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("");
  }

  public RawFiducial getFiducialWithId(int id) {
    for (RawFiducial fiducial : fiducials) {
      if (fiducial.id != id) {
        continue;
      }

      return fiducial;
    }
    throw new NoSuchTargetException("No target with ID " + id + " is in view!");
  }
}