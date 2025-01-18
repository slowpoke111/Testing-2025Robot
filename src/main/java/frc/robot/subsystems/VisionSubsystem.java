package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {
        return;
    }
    
    
    public void periodic() {
        return;
      }

    public double getDistance(int pipelineID, double goalHeight){
        LimelightHelpers.setPipelineIndex("",pipelineID);
        double targetAngleOffset = LimelightHelpers.getTY("");
    
        double limelightMountAngle = VisionConstants.LIMELIGHT_ANGLE; 
    
        double lensHeight = VisionConstants.LIMELIGHT_LENS_HEIGHT; //in  

        //Convert to radians
        double angleToGoalDegrees = targetAngleOffset + limelightMountAngle;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    
        //calculate distance
        double distance = (goalHeight - lensHeight) / Math.tan(angleToGoalRadians);
        return distance;
    }

    public double getTX(String limelightName){
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTY(String limelightName){
        return LimelightHelpers.getTY(limelightName);
    }

    

}


