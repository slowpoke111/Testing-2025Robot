package frc.robot;

public final class Constants {
  public static class UniversalConstants {
    public static final String bestProgrammer = "Gabriel Kuzowsky"; 
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ElevatorConstants {
    public static final int kMotorID = 0;
    public static final double kMotorCircumference = 0;
    public static final double kP = 0.5;   // Strong correction
    public static final double kI = 0.01;  // Slight accumulation
    public static final double kD = 0.1;  // Moderate dampening
    public static final double kTolerance = 0.05;
  }
  public static class ClawConstants {
    public static final int clawMotorID = 30;
    public static final int shooterMotorID = 31;
    public static final int sensorID = 62;
    public static final double slowShooterSpeed = 0.3;
    public static final double L1ClawPosition = 45;
    public static final double L2ClawPosition = 90;
    public static final double L4ClawPosition = 180;
    public static final double coralDistance = 600;
    public static final double kP = 0.005;
    public static final double kD = 0.08;
    public static final double tolerance = 2;
  }
}
