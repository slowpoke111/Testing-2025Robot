package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightHelpers.*;

public class VisionSubsystem extends SubsystemBase {
  private RawFiducial[] fiducials;
  private final LEDSubsystem m_LEDs = new LEDSubsystem();

  public VisionSubsystem() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException { //No fiducial fonund
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {
    //Enable if fps is an issue
    // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

    LimelightHelpers.setCameraPose_RobotSpace( // maybe put in consts.java
        "",
        0.26035, 
        0.3175,
        0.3048,
        0,
        -0.5,
        0);
    LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] {0,1,3,5,6,8,9,10,11,12});

    SmartDashboard.putNumber("Rotate P",0.0);
    SmartDashboard.putNumber("Rotate D",0.0);

  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("");
    SmartDashboard.putBoolean("Is aligned:", isAligned());
    if (isAligned()){
      m_LEDs.runLEDs(0.73);
    }
    else {
      m_LEDs.runLEDs(0.87);
    }
  }
  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;
    //Linear search for close
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }
    return closest;
  }

  //Linear searcgh by id
  public RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

public RawFiducial getFiducialWithId(int id, boolean verbose) {//Debug
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

  public boolean isAligned(){
    if (Math.abs(getTX()-VisionConstants.branchAngle)<VisionConstants.branchTolerance){return true;} //Left
    if (Math.abs(getTX()+VisionConstants.branchAngle)<VisionConstants.branchTolerance){return true;} //Right
    return false;
  }

  //Get values
  public double getTX(){
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTY(){
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
  }

  public Angle getTXAngle(){
    return Angle.ofBaseUnits(LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME), Degrees);
  }
  public Angle getTYAngle(){
    return Angle.ofBaseUnits(LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME), Degrees);
  }

  public double getTA(){
    return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME);
  }
  public boolean getTV(){
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
  }

  public double getClosestTX(){
    return getClosestFiducial().txnc;
  }
  public double getClosestTY(){
    return getClosestFiducial().tync;
  }
  public double getClosestTA(){
    return getClosestFiducial().ta;
  }

  public double getID_TX(int ID){
    return getFiducialWithId(ID).txnc;
  }
  public double getID_TY(int ID){
    return getFiducialWithId(ID).tync;
  }
}
