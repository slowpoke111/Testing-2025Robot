package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
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

    public Distance getDistance(int pipelineID, double goalHeight) {
        setPipelineIndex(pipelineID);

        Angle angleToGoal = getTY().plus(VisionConstants.LIMELIGHT_ANGLE);

        Distance lensHeight = VisionConstants.LIMELIGHT_LENS_HEIGHT;

        double distance = (goalHeight - lensHeight.in(Inches)) / Math.tan(angleToGoal.in(Radians));
        Distance distanceUnit = Distance.ofBaseUnits(distance, Inches);
        return distanceUnit;
    }


    public void setPipelineIndex(int i){
        m_limelightNT.getEntry("pipeline").setNumber(i);
    }

    public Angle getTX(){
        return Angle.ofBaseUnits(m_limelightNT.getEntry("tx").getDouble(-1000.0), Degrees);
    }

    public Angle getTY(){
        return Angle.ofBaseUnits(m_limelightNT.getEntry("ty").getDouble(-1000.0), Degrees);
    }

    public Dimensionless getTA(){
        return Dimensionless.ofBaseUnits(m_limelightNT.getEntry("ta").getDouble(-1), Percent);
    }

    public boolean isTarget(){
        return m_limelightNT.getEntry("tv").getBoolean(false);
    }

    

}


