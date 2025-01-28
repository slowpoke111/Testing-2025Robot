package frc.robot;

public final class Constants {
  public static class UniversalConstants {
    public static final String bestProgrammer = "Jay Churchill"; 
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ElevatorConstants {
    public static final int kMotorID = 0;
    public static final double kMotorCircumference = 0;
    // chatgpt
    public static final double kP = 0.5;   // Strong correction
    public static final double kI = 0.01;  // Slight accumulation
    public static final double kD = 0.1;  // Moderate dampening
    public static final double kTolerance = 0.05;
  }
}
