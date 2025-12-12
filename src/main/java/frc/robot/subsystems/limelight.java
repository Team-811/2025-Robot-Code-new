package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Thin Limelight subsystem wrapper. Reads NetworkTables outputs once per loop, exposes safe getters
 * for target pose components, and provides convenience helpers for simple aim/drive duty cycles.
 * Keep vision math here so commands stay lean and reusable.
 */
public class Limelight extends SubsystemBase {
  // NetworkTables entries (avoids re-looking them up every loop)
  private final NetworkTable table;
  private final NetworkTableEntry tvEntry;         // target valid (0 or 1)
  private final NetworkTableEntry tidEntry;        // target id
  private final NetworkTableEntry targetPoseEntry; // targetpose_cameraspace -> double[6]

  // Internal cached state (filled each periodic)
  private final double[] targetPose = new double[6];
  private int targetId = -1;
  private final Timer timer = new Timer();        // example timer to track target age
  private boolean isTargeting = false;            // toggle hook if you want to gate outputs
  private boolean lastHasTarget = false;
  private double lastUpdateTimeSeconds = 0.0; // toggle hook if you want to gate outputs

  // Tuning constants
  private static final double DEFAULT_SIDE_OFFSET = 0.1; // meters offset used for left/right aim
  private static final double DUTY_CLAMP = 0.8;          // max magnitude for duty outputs
  private static final double STALE_TARGET_TIMEOUT_S = 0.25; // zero outputs if data older than this          // max magnitude for duty outputs

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tvEntry = table.getEntry("tv");
    tidEntry = table.getEntry("tid");
    targetPoseEntry = table.getEntry("targetpose_cameraspace");

    // Start with zeros so getters are safe before data arrives.
    for (int i = 0; i < targetPose.length; i++) {
      targetPose[i] = 0.0;
    }

    // If you need tag filtering/orientation, configure it here (uncomment after verifying API):
    // int[] validIDs = {3, 4};
    // LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    // LimelightHelpers.SetRobotOrientation("limelight", yawDeg, yawRate, pitchDeg, pitchRate, rollDeg, rollRate);
  }

  @Override
  public void periodic() {
    // Read validity flag and pose once per loop. tv: 1 = valid target, 0 = none.
    double tv = tvEntry.getDouble(0.0);
    lastHasTarget = tv >= 0.5;

    // Copy pose data defensively (array may be missing or shorter than expected).
    double[] rawPose = targetPoseEntry.getDoubleArray(new double[0]);
    for (int i = 0; i < targetPose.length; i++) {
      targetPose[i] = (rawPose != null && i < rawPose.length) ? rawPose[i] : 0.0;
    }

    targetId = (int) tidEntry.getNumber(-1).doubleValue();

    // Example timer usage: track how long a target has been visible and age out stale data.
    if (lastHasTarget) {
      if (!timer.isRunning()) {
        timer.reset();
        timer.start();
      }
      lastUpdateTimeSeconds = Timer.getFPGATimestamp();
    } else {
      timer.stop();
    }
  }

  // ---------------------------
  // Basic getters (safe)
  // ---------------------------

    /** True when the Limelight reports a valid target. */
  public boolean hasTarget() {
    return lastHasTarget;
  }

  private boolean isStale() {
    return (Timer.getFPGATimestamp() - lastUpdateTimeSeconds) > STALE_TARGET_TIMEOUT_S;
  }

  /** X (lateral) camera-space meters; returns 0 when no target. */
  public double getX() {
    if (isStale()) return 0.0;
    return hasTarget() ? targetPose[0] : 0.0;
  }

  /** Z (forward) camera-space meters; returns 0 when no target. */
  public double getZ() {
    if (isStale()) return 0.0;
    return hasTarget() ? targetPose[2] : 0.0;
  }

  /** Yaw (degrees in most pipelines) converted to radians; returns 0 when no target. */
  public double getYawRadians() {
    if (!isTargeting || !hasTarget() || isStale()) return 0.0;
    double angleDeg = targetPose.length > 4 ? targetPose[4] : 0.0;
    return Math.toRadians(angleDeg);
  }

  public int getTargetId() {
    return targetId;
  }

  /** Camera Y (vertical) meters; returns 0 when no target. */
  public double getMeasureY() {
    if (isStale()) return 0.0;
    return hasTarget() && targetPose.length > 1 ? targetPose[1] : 0.0;
  }

  /** Alias for X to preserve compatibility with prior code. */
  public double getMeasureX() {
    return getX();
  }

  public double getLeftX() {
    return hasTarget() ? getX() - DEFAULT_SIDE_OFFSET : 0.0;
  }

  public double getRightX() {
    return hasTarget() ? getX() + DEFAULT_SIDE_OFFSET : 0.0;
  }

  // ---------------------------
  // Aiming / duty-cycle helpers
  // ---------------------------

  public double targetXError() {
    return getX();
  }

  public double AimTargetXDutyCycle() {
    if (!isTargeting || !hasTarget() || isStale()) return 0.0;
    return MathUtil.clamp(targetXError(), -DUTY_CLAMP, DUTY_CLAMP);
  }

  public double targetZError() {
    return getZ();
  }

  public double AimTargetZDutyCycle() {
    if (!isTargeting || !hasTarget() || isStale()) return 0.0;
    return MathUtil.clamp(targetZError(), -DUTY_CLAMP, DUTY_CLAMP);
  }

  public double targetYawError() {
    // Negate to match original sign convention (rotate to reduce yaw error).
    return -getYawRadians();
  }

  public double AimTargetYawDutyCycle() {
    if (!isTargeting || !hasTarget() || isStale()) return 0.0;
    return MathUtil.clamp(targetYawError() * 1.05, -DUTY_CLAMP, DUTY_CLAMP);
  }

  public double RobotXDutyCycle() {
    if (!isTargeting || !hasTarget() || isStale()) return 0.0;
    double yaw = getYawRadians();
    double xDuty = AimTargetXDutyCycle();
    double zDuty = AimTargetZDutyCycle();
    double target = (-Math.sin(yaw) * xDuty) + (Math.cos(yaw) * zDuty);
    return MathUtil.clamp(target, -DUTY_CLAMP, DUTY_CLAMP);
  }

  public double RobotYDutyCycle() {
    if (!isTargeting || !hasTarget() || isStale()) return 0.0;
    double yaw = getYawRadians();
    double xDuty = AimTargetXDutyCycle();
    double zDuty = AimTargetZDutyCycle();
    double target = (Math.cos(yaw) * xDuty) + (Math.sin(yaw) * zDuty);
    return MathUtil.clamp(target, -DUTY_CLAMP, DUTY_CLAMP);
  }

  // ---------------------------
  // Targeting state hook
  // ---------------------------

  /** Optional toggle; gate your commands on this if you want manual enable/disable. */
  public void setTargeting(boolean targetingState) {
    isTargeting = targetingState;
    System.out.println("Limelight targeting: " + isTargeting);
    if (!isTargeting) {
      timer.stop();
    } else {
      timer.reset();
      timer.start();
    }
  }
}





