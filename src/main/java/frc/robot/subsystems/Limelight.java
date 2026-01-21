// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// /**
//  * Minimal Limelight helper for AprilTag targeting.
//  * Reads NT entries each loop (tx/ty/ta + camera-space pose), computes simple distance/height
//  * estimates, and publishes telemetry for debugging.
//  */
// public class Limelight extends SubsystemBase {
//   private static final String TABLE_NAME = "limelight"; // change if your LL name differs
//   private final NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);

//   // Cached values (meters/degrees) for telemetry and commands.
//   private double tx;
//   private double ty;
//   private double ta;
//   private double targetPoseZ;
//   private double targetPoseX;
//   private double targetPoseY;
//   private double targetPoseYaw;
//   private boolean hasTarget;

//   @Override
//   public void periodic() {
//     // Poll Limelight NetworkTables entries. If no target, LL reports zeros.
//     tx = table.getEntry("tx").getDouble(0.0);
//     ty = table.getEntry("ty").getDouble(0.0);
//     ta = table.getEntry("ta").getDouble(0.0);
//     // targetpose_cameraspace = 3D pose of the detected tag relative to the camera (meters, radians).
//     double[] pose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
//     if (pose.length >= 6) {
//       targetPoseX = pose[0];
//       targetPoseY = pose[1];
//       targetPoseZ = pose[2];
//       targetPoseYaw = pose[4];
//     }
//     hasTarget = table.getEntry("tv").getDouble(0) == 1.0;
//     publishTelemetry();
//   }

//   // private void publishTelemetry() {
//   //   // Push values to dashboard for tuning/driver feedback.
//   //   SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget);
//   //   SmartDashboard.putNumber("Limelight/tx", tx);
//   //   SmartDashboard.putNumber("Limelight/ty", ty);
//   //   SmartDashboard.putNumber("Limelight/ta", ta);
//   //   SmartDashboard.putNumber("Limelight/TargetPoseX_m", targetPoseX);
//   //   SmartDashboard.putNumber("Limelight/TargetPoseY_m", targetPoseY);
//   //   SmartDashboard.putNumber("Limelight/TargetPoseZ_m", targetPoseZ);
//   //   SmartDashboard.putNumber("Limelight/TargetPoseYaw_deg", targetPoseYaw);
//   //   SmartDashboard.putNumber("Limelight/Range_m", getRangeMeters());
//   //   SmartDashboard.putNumber("Limelight/HeightOffset_m", getHeightOffsetMeters());
//   // }
//   private void publishTelemetry() {
//     NetworkTable table =
//         NetworkTableInstance.getDefault().getTable("Elastic");

//     table.getEntry("Limelight/HasTarget").setBoolean(hasTarget);
//     table.getEntry("Limelight/tx").setDouble(tx);
//     table.getEntry("Limelight/ty").setDouble(ty);
//     table.getEntry("Limelight/ta").setDouble(ta);

//     table.getEntry("Limelight/TargetPoseX_m").setDouble(targetPoseX);
//     table.getEntry("Limelight/TargetPoseY_m").setDouble(targetPoseY);
//     table.getEntry("Limelight/TargetPoseZ_m").setDouble(targetPoseZ);
//     table.getEntry("Limelight/TargetPoseYaw_deg").setDouble(targetPoseYaw);

//     table.getEntry("Limelight/Range_m").setDouble(getRangeMeters());
//     table.getEntry("Limelight/HeightOffset_m").setDouble(getHeightOffsetMeters());
// }


//   public boolean hasTarget() { return hasTarget; }
//   public double getTx() { return tx; }
//   public double getTy() { return ty; }
//   public double getTargetPoseZ() { return targetPoseZ; }
//   public double getTargetPoseX() { return targetPoseX; }
//   public double getTargetPoseY() { return targetPoseY; }
//   public double getTargetPoseYaw() { return targetPoseYaw; }

//   /** Approximate straight-line distance from camera to target (meters) using camera-space pose. */
//   public double getRangeMeters() { return Math.hypot(targetPoseX, targetPoseZ); }

//   /** Approximate vertical offset (meters) from camera to target center. */
//   public double getHeightOffsetMeters() { return targetPoseY; }
// }
