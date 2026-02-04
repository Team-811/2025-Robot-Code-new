import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.CANdleControlFrame;
import com.ctre.phoenix6.configs.CANdleConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  // Set your CAN ID
  private final CANdle candle = new CANdle(0); // CAN ID 0 example

  private final NetworkTable limelight =
      NetworkTableInstance.getDefault().getTable("limelight");

  @Override
  public void robotInit() {
    // Optional: configure CANdle behavior (safe defaults)
    CANdleConfiguration cfg = new CANdleConfiguration();
    candle.getConfigurator().apply(cfg);

    // Optional: reduce CAN bandwidth used by LED updates
    candle.setControlFramePeriod(CANdleControlFrame.Control_1_General, 20);
  }

  @Override
  public void robotPeriodic() {
    double tv = limelight.getEntry("tv").getDouble(0); // 0 or 1
    boolean hasTarget = (tv == 1.0);

    if (hasTarget) {
      // Solid green on all 8 onboard LEDs
      candle.setLEDs(0, 255, 0); // R,G,B
    } else {
      // Off
      candle.setLEDs(0, 0, 0);
    }
  }
}
