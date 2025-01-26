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
        System.out.println("TX: " + TX_value + ", TY: " + TY_value);
    }

    public Distance getDistance(int pipelineID, double goalHeight) {

       Angle angleToGoal = Angle.ofBaseUnits(getTY(),Degrees).plus(VisionConstants.LIMELIGHT_ANGLE);

       Distance lensHeight = VisionConstants.LIMELIGHT_LENS_HEIGHT;

       double distance = (goalHeight - lensHeight.in(Inches)) / Math.tan(angleToGoal.in(Radians));
       Distance distanceUnit = Distance.ofBaseUnits(distance, Inches);
       return distanceUnit;
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


