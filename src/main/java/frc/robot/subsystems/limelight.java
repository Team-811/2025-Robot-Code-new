// package frc.robot.subsystems;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// /**
//  * Thin Limelight subsystem wrapper. Reads NetworkTables outputs once per loop, exposes safe getters
//  * for target pose components, and provides convenience helpers for simple aim/drive duty cycles.
//  * Keep vision math here so commands stay lean and reusable.
//  */
// public class Limelight extends SubsystemBase {
//   private static final String DEFAULT_TABLE = "limelight"; // camera NT name

//   // NetworkTables entries (avoids re-looking them up every loop)
//   private final NetworkTable table;
//   private final NetworkTableEntry tvEntry;         // target valid (0 or 1)
//   private final NetworkTableEntry tidEntry;        // target id
//   private final NetworkTableEntry targetPoseEntry; // targetpose_cameraspace -> double[6]
//   private final NetworkTableEntry pipelineEntry;   // active pipeline (read/write)
//   private final NetworkTableEntry ledModeEntry;    // LED mode (write)
//   private final NetworkTableEntry latencyEntry;    // pipeline latency (ms)

//   // Internal cached state (filled each periodic)
//   private final double[] targetPose = new double[6];
//   private int targetId = -1;
//   private final Timer timer = new Timer();        // example timer to track target age
//   private final Timer lossTimer = new Timer();    // debounce loss of target
//   private boolean isTargeting = false;            // toggle hook if you want to gate outputs
//   private boolean lastHasTarget = false;
//   private double lastUpdateTimeSeconds = 0.0; // toggle hook if you want to gate outputs

//   // Tuning constants
//   private static final double DEFAULT_SIDE_OFFSET = 0.1; // meters offset used for left/right aim
//   private static final double DUTY_CLAMP = 0.8;          // max magnitude for duty outputs
//   private static final double STALE_TARGET_TIMEOUT_S_DEFAULT = 0.5; // zero outputs if data older than this
//   private static final double LOSS_DEBOUNCE_S_DEFAULT = 0.3; // require sustained loss before declaring no target
//   private static final double POSE_DEADBAND_METERS_DEFAULT = 0.02; // ignore tiny pose jitters
//   private static final double YAW_DEADBAND_RAD_DEFAULT = 0.02;     // ignore tiny yaw jitters
//   private double staleTargetTimeout = STALE_TARGET_TIMEOUT_S_DEFAULT;
//   private double lossDebounceSeconds = LOSS_DEBOUNCE_S_DEFAULT;
//   private double poseDeadbandMeters = POSE_DEADBAND_METERS_DEFAULT;
//   private double yawDeadbandRad = YAW_DEADBAND_RAD_DEFAULT;
//   private static final double MIN_POSE_Z_METERS = 0.1; // minimum believable forward distance

//   public Limelight() {
//     this(DEFAULT_TABLE);
//   }

//   public Limelight(String tableName) {
//     table = NetworkTableInstance.getDefault().getTable(tableName);
//     tvEntry = table.getEntry("tv");
//     tidEntry = table.getEntry("tid");
//     targetPoseEntry = table.getEntry("targetpose_cameraspace");
//     pipelineEntry = table.getEntry("getpipe");
//     ledModeEntry = table.getEntry("ledMode");
//     latencyEntry = table.getEntry("tl"); // total latency ms

//     // Start with zeros so getters are safe before data arrives.
//     for (int i = 0; i < targetPose.length; i++) {
//       targetPose[i] = 0.0;
//     }

//     SmartDashboard.putNumber("Limelight/Tune/StaleTimeoutS", staleTargetTimeout);
//     SmartDashboard.putNumber("Limelight/Tune/LossDebounceS", lossDebounceSeconds);
//     SmartDashboard.putNumber("Limelight/Tune/PoseDeadbandM", poseDeadbandMeters);
//     SmartDashboard.putNumber("Limelight/Tune/YawDeadbandRad", yawDeadbandRad);

//     // If you need tag filtering/orientation, configure it here (uncomment after verifying API):
//     // int[] validIDs = {3, 4};
//     // LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
//     // LimelightHelpers.SetRobotOrientation("limelight", yawDeg, yawRate, pitchDeg, pitchRate, rollDeg, rollRate);
//   }

//   @Override
//   public void periodic() {
//     staleTargetTimeout = SmartDashboard.getNumber("Limelight/Tune/StaleTimeoutS", staleTargetTimeout);
//     lossDebounceSeconds = SmartDashboard.getNumber("Limelight/Tune/LossDebounceS", lossDebounceSeconds);
//     poseDeadbandMeters = SmartDashboard.getNumber("Limelight/Tune/PoseDeadbandM", poseDeadbandMeters);
//     yawDeadbandRad = SmartDashboard.getNumber("Limelight/Tune/YawDeadbandRad", yawDeadbandRad);

//     double tv = tvEntry.getDouble(0.0);
//     boolean rawHasTarget = tv >= 0.5;

//     // Update timestamp first to keep timing consistent
//     if (rawHasTarget) {
//       lastUpdateTimeSeconds = Timer.getFPGATimestamp();
//     }

//     double[] rawPose = targetPoseEntry.getDoubleArray(new double[0]);
//     for (int i = 0; i < targetPose.length; i++) {
//       targetPose[i] = (i < rawPose.length) ? rawPose[i] : 0.0;
//     }

//     targetId = (int) tidEntry.getNumber(-1).doubleValue();

//     if (rawHasTarget) {
//       lossTimer.stop();
//       lossTimer.reset();
//       lastHasTarget = true;
//       if (!timer.isRunning()) {
//         timer.reset();
//         timer.start();
//       }
//     } else {
//       if (!lossTimer.isRunning()) {
//         lossTimer.reset();
//         lossTimer.start();
//       }
//       if (lossTimer.hasElapsed(lossDebounceSeconds)) {
//         lastHasTarget = false;
//       }
//       timer.stop();
//     }

//     SmartDashboard.putBoolean("Limelight/HasTarget", lastHasTarget);
//     SmartDashboard.putNumber("Limelight/TargetId", targetId);
//     SmartDashboard.putNumber("Limelight/Pipeline", pipelineEntry.getDouble(-1));
//     SmartDashboard.putNumber("Limelight/LatencyMs", latencyEntry.getDouble(0.0));
//     SmartDashboard.putNumber("Limelight/CamX", getX());
//     SmartDashboard.putNumber("Limelight/CamZ", getZ());
//     SmartDashboard.putNumber("Limelight/YawRad", getYawRadians());
//     SmartDashboard.putNumber("Limelight/LastUpdateSec", lastUpdateTimeSeconds);
//     SmartDashboard.putBoolean("Limelight/Stale", isStale());
//     SmartDashboard.putBoolean("Limelight/PoseSuspicious", lastHasTarget && !isPoseValid());
//   }

//   // ---------------------------
//   // Basic getters (safe)
//   // ---------------------------

//     /** True when the Limelight reports a valid target. */
//   public boolean hasTarget() {
//     return lastHasTarget;
//   }

//   private boolean isStale() {
//     return (Timer.getFPGATimestamp() - lastUpdateTimeSeconds) > staleTargetTimeout;
//   }

//   /**
//    * X (lateral) camera-space meters; returns 0 when no target.
//    * Limelight targetpose_cameraspace axes: X=forward, Y=left/right, Z=up/down.
//    * We expose "X" here as the left/right component for backward compatibility.
//    */
//   public double getX() {
//     if (isStale()) return 0.0;
//     return hasTarget() && targetPose.length > 1 ? targetPose[1] : 0.0; // Y axis from Limelight
//   }

//   /** Z (forward) camera-space meters; returns 0 when no target. (Limelight X axis) */
//   public double getZ() {
//     if (isStale()) return 0.0;
//     return hasTarget() ? targetPose[0] : 0.0; // X axis from Limelight
//   }

//   /** True when pose data looks reasonable (non-zero forward distance) while target is valid. */
//   public boolean isPoseValid() {
//     return hasTarget() && !isStale() && Math.abs(targetPose[0]) > MIN_POSE_Z_METERS;
//   }

//   /** Yaw (degrees in most pipelines) converted to radians; returns 0 when no target. */
//   public double getYawRadians() {
//     if (!isTargeting || !hasTarget() || isStale()) return 0.0;
//     double angleDeg = targetPose.length > 4 ? targetPose[4] : 0.0;
//     return Math.toRadians(angleDeg);
//   }

//   public int getTargetId() {
//     return targetId;
//   }

//   /** Camera Y (vertical) meters; returns 0 when no target. */
//   public double getMeasureY() {
//     if (isStale()) return 0.0;
//     return hasTarget() && targetPose.length > 2 ? targetPose[2] : 0.0;
//   }

//   /** Alias for X to preserve compatibility with prior code. */
//   public double getMeasureX() {
//     return getX();
//   }

//   public double getLeftX() {
//     if (isStale()) return 0.0;
//     return hasTarget() ? getX() - DEFAULT_SIDE_OFFSET : 0.0;
//   }

//   public double getRightX() {
//     if (isStale()) return 0.0;
//     return hasTarget() ? getX() + DEFAULT_SIDE_OFFSET : 0.0;
//   }

//   // ---------------------------
//   // Aiming / duty-cycle helpers
//   // ---------------------------

//   public double targetXError() {
//     return MathUtil.applyDeadband(getX(), poseDeadbandMeters);
//   }

//   public double AimTargetXDutyCycle() {
//     if (!isTargeting || !hasTarget() || isStale()) return 0.0;
//     return MathUtil.clamp(targetXError(), -DUTY_CLAMP, DUTY_CLAMP);
//   }

//   public double targetZError() {
//     return MathUtil.applyDeadband(getZ(), poseDeadbandMeters);
//   }

//   public double AimTargetZDutyCycle() {
//     if (!isTargeting || !hasTarget() || isStale()) return 0.0;
//     return MathUtil.clamp(targetZError(), -DUTY_CLAMP, DUTY_CLAMP);
//   }

//   public double targetYawError() {
//     // Negate to match original sign convention (rotate to reduce yaw error).
//     return MathUtil.applyDeadband(-getYawRadians(), yawDeadbandRad);
//   }

//   public double AimTargetYawDutyCycle() {
//     if (!isTargeting || !hasTarget() || isStale()) return 0.0;
//     return MathUtil.clamp(targetYawError() * 1.05, -DUTY_CLAMP, DUTY_CLAMP);
//   }

//   public double RobotXDutyCycle() {
//     if (!isTargeting || !hasTarget() || isStale()) return 0.0;
//     double yaw = getYawRadians();
//     double xDuty = AimTargetXDutyCycle();
//     double zDuty = AimTargetZDutyCycle();
//     double target = (-Math.sin(yaw) * xDuty) + (Math.cos(yaw) * zDuty);
//     return MathUtil.clamp(target, -DUTY_CLAMP, DUTY_CLAMP);
//   }

//   public double RobotYDutyCycle() {
//     if (!isTargeting || !hasTarget() || isStale()) return 0.0;
//     double yaw = getYawRadians();
//     double xDuty = AimTargetXDutyCycle();
//     double zDuty = AimTargetZDutyCycle();
//     double target = (Math.cos(yaw) * xDuty) + (Math.sin(yaw) * zDuty);
//     return MathUtil.clamp(target, -DUTY_CLAMP, DUTY_CLAMP);
//   }

//   // ---------------------------
//   // Targeting state hook
//   // ---------------------------

//   /** Optional toggle; gate your commands on this if you want manual enable/disable. */
//   public void setTargeting(boolean targetingState) {
//     isTargeting = targetingState;
//     System.out.println("Limelight targeting: " + isTargeting);
//     if (!isTargeting) {
//       timer.stop();
//       lossTimer.stop();
//       lossTimer.reset();
//     } else {
//       timer.reset();
//       timer.start();
//       lossTimer.stop();
//       lossTimer.reset();
//     }
//   }

//   // ---------------------------
//   // Pipeline / LED helpers
//   // ---------------------------

//   /** Set the active pipeline (0-based). */
//   public void setPipeline(int pipeline) {
//     table.getEntry("pipeline").setNumber(pipeline);
//     pipelineEntry.setNumber(pipeline); // Also update read entry for consistency
//   }

//   /** Force LEDs on/off (true = on, false = off). */
//   public void setLEDs(boolean on) {
//     ledModeEntry.setNumber(on ? 3 : 1); // 3 = force on, 1 = force off
//   }
// }
