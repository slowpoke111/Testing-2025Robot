package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.security.Timestamp;
import java.text.NumberFormat.Style;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ser.std.StringSerializer;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;

    private double TX_value;
    private double TY_value;

    public VisionSubsystem(DoubleSupplier txSupplier, DoubleSupplier tySupplier) {
        this.txSupplier = txSupplier;
        this.tySupplier = tySupplier;
    }

    public void periodic() {
        this.TX_value = txSupplier.getAsDouble();
        this.TY_value = tySupplier.getAsDouble();
    }

    public double getDistance(double goalHeight) {
        double targetOffsetAngle_Vertical = getTY();

        double limelightMountAngleDegrees = 45.0; 

        double limelightLensHeightInches = 33.0; 
    
        double goalHeightInches = 36.0; 
    
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    
        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }


    public void setPipelineIndex(int i){};

    public double getTX() {
        return this.TX_value;
    }
    
    public double getTY() {
        return this.TY_value;
    }

    public Dimensionless getTA(){
        return Dimensionless.ofBaseUnits(0.0, Percent);
    }

    public boolean isTarget(){
        return false;
    }

    public String getKeys(){
        return NetworkTableInstance.getDefault().getTable("limelight").getKeys().toString();
    }
}