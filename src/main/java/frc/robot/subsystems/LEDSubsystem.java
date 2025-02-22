package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  private Spark ledDriverOne;

  public LEDSubsystem() {
    ledDriverOne = new Spark(Constants.LEDConstants.LEDDriverOneID);
  }

  public void LEDColor(double color) {
    ledDriverOne.set(color);
  }
}