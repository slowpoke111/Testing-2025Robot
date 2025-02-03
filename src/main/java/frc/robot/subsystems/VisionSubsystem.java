package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
    LimelightHelpers.setCameraPose_RobotSpace(
        "",
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
    StringBuilder availableIds = new StringBuilder();
  
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

public RawFiducial getFiducialWithId(int id, boolean verbose) {
  StringBuilder availableIds = new StringBuilder();

  for (RawFiducial fiducial : fiducials) {
      if (availableIds.length() > 0) {
          availableIds.append(", ");
      } //Error reporting
      availableIds.append(fiducial.id);
      
      if (fiducial.id == id) {
          return fiducial;
      }
  }
  throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
  }
}
