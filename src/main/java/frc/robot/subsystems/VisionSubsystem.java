package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTableInstance m_NetworkTableInstance;
    private final NetworkTable m_limelightNT;
    public VisionSubsystem() {
        m_NetworkTableInstance = NetworkTableInstance.getDefault();
        m_limelightNT = m_NetworkTableInstance.getTable("limelight-a");

    }
    
    public void periodic() {
        return;
      }

    public double getDistance(int pipelineID, double goalHeight){
        setPipelineIndex(pipelineID);
        double targetAngleOffset = getTY();
        double limelightMountAngle = VisionConstants.LIMELIGHT_ANGLE; 
    
        double lensHeight = VisionConstants.LIMELIGHT_LENS_HEIGHT; //in  

        //Convert to radians
        double angleToGoalDegrees = targetAngleOffset + limelightMountAngle;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    
        //calculate distance
        double distance = (goalHeight - lensHeight) / Math.tan(angleToGoalRadians);
        return distance;
    }

    public void setPipelineIndex(int i){
        m_limelightNT.getEntry("pipeline").setNumber(i);
    }

    public double getTX(){
        return m_limelightNT.getEntry("tx").getDouble(-1000.0);
    }

    public double getTY(){
        return m_limelightNT.getEntry("ty").getDouble(-1000.0);
    }

    public double getTA(){
        return m_limelightNT.getEntry("ta").getDouble(-1);
    }

    public boolean isTarget(){
        return m_limelightNT.getEntry("tv").getBoolean(false);
    }

    

}


