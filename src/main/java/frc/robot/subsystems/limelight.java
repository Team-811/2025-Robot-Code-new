
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import edu.wpi.first.apriltag.AprilTagDetection;
// import edu.wpi.first.apriltag.AprilTagDetector;
// import edu.wpi.first.apriltag.AprilTagPoseEstimator;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.networktables.IntegerArrayPublisher;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
// import edu.wpi.first.math.geometry.Pose2d;

// import java.util.ArrayList;
// import org.opencv.core.Mat;
// import org.opencv.core.Point;
// import org.opencv.core.Scalar;
// import org.opencv.imgproc.Imgproc;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.estimator.PoseEstimator;

// /**
//  * This is a demo program showing the detection of AprilTags. The image is acquired from the USB
//  * camera, then any detected AprilTags are marked up on the image and sent to the dashboard.
//  *
//  * <p>Be aware that the performance on this is much worse than a coprocessor solution!
//  */
// public class limelight extends SubsystemBase {
//   // static PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
//   /** Called once at the beginning of the robot program. */
//   NetworkTable table;
//   // private double x;
//   private long v;
//   double[] targetPose;
//   private long targetId;
//   // double leftOffset;
//   // double rightOffset;
//  private Timer timer;
//  boolean isTargeting = false;
//  boolean timerIsStarted = false;
//  Distance positionX;
//  NetworkTableEntry tv;
//  Distance positionY;
//  double targetCenter;
//  Pose2d myPose = new Pose2d();



//   public limelight() {
//     // var visionThread = new Thread(this::apriltagVisionThreadProc);
//     // visionThread.setDaemon(true);
//     // visionThread.start();
//     timer = new Timer();
//     positionX = myPose.getMeasureX();
//     positionY = myPose.getMeasureY();
//     int[] validIDs = {3,4};
//     LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
//     LimelightHelpers.SetRobotOrientation("limelight", 20, targetCenter, targetCenter, v, targetId, targetCenter);
//   }
//   public void setTargeting(boolean targetingState){
//     isTargeting= targetingState;
//     System.out.println(isTargeting);

//   }
// public void periodic(){
//  table = NetworkTableInstance.getDefault().getTable("limelight");
//   // NetworkTableEntry tx = table.getEntry("tx");
//  tv = table.getEntry("tv");

//   //  x = tx.getDouble(0);
//    v = tv.getInteger(0);
//    targetPose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
//     targetId = table.getEntry("tid").getInteger(0);
//        // System.out.println(valid);
//       //  System.out.println("id: "+targetId);
//       // leftOffset=0;
//       // rightOffset=0;

// }
// public static double getDisatnce(){
//   double height = 0;
//   double targetHight = 0;
//   double angle = 0;
//   double targetAngle =0;
//   return(targetHight - height)/Math.tan(Math.toRadians(angle + targetAngle));
// }
// public double getX(){
//   // System.out.println(x);
//   return targetPose[0];
// }
// public double getV(){
//   // System.out.println(v);

//   return v;
// }
// public double getYaw(){
//   // System.out.println(targetPose[4]);
//   // return targetPose[4]*v;
//   return targetPose[4] * Math.PI/180;
// }
// public double getY(){
//   return 0;
// }
// public double getZ(){
//   return targetPose[2];
// }
// public double getLeftX(){
//   return targetPose[0]-0.1;
// //   System.out.println("timer "+timer.get());
// //   System.out.println("v" + v);
// //   System.out.println("timer is started: " +timerIsStarted);
// //   if(!isTargeting)
// //   return 0;
// //   System.out.println("hi");
// //   if(targetPose[0] - 0.1 < 0.01 && targetPose[0]-0.1>-0.01)
// //   return 0;
// //   if(v==0 && !timerIsStarted){
// //     System.out.println("hello");
// //     timerIsStarted = true;
// //   timer.start();
// //   timer.reset();
// //   return 0;
// // }
// //   else if(timer.get()<0.5){
// //     return targetPose[0]-0.1;
// //   }
// //   else{
// //     timer.stop();
// //     timerIsStarted = false;
// //     isTargeting = false;
// //     return 0 ;
// //   }


//   // return targetPose[0]-0.1;
// }
// public double getRightX(){
//   return targetPose[0]+0.1;
// }
// public Distance getMeasureY(){
//   System.out.println(positionX);
//  return positionX;

// }
// public Distance getMeasureX(){
//   System.out.println(positionY);
// return positionY;
// }
// public boolean hasTarget(){
//   return tv.getDouble(0) ==1;
// }
// public double targetXError() {
//   return getX();
//  }
// public double AimTargetXDutyCycle(){
//   if (! hasTarget()) {
//     return 0;
//   }
//   double target;
//   double Error =  targetXError();


//     target= MathUtil.clamp(
//  (
//     Error

//   ),-0.8,0.8); 


 
//  return target;
 
//  }
//  public double targetZError(){
//   return getZ();
// }
// public double targetYawError() {
//   return -getYaw();
//  }
//  public double AimTargetZDutyCycle(){
//   if (! hasTarget()) return 0;
// double target =
//  MathUtil.clamp(targetZError(),-0.8,0.8);


//   return 
   
//   target;
//  }
// public double RobotXDutyCycle(){
//   if (!hasTarget()) return 0;
//  double target = MathUtil.clamp((-Math.sin(getYaw())*AimTargetXDutyCycle())+(Math.cos(getYaw())
 
//  *AimTargetZDutyCycle()),-.8,.8);

//  return target;
// }

// public double AimTargetYawDutyCycle(){
//   if (! hasTarget()) return 0;
//   double target = MathUtil.clamp( ((targetYawError())*1.05),-0.8,0.8);
//   return target;
// }

// public double RobotYDutyCycle(){
//   if (!hasTarget()) return 0;
//   double target = MathUtil.clamp((Math.cos(getYaw())*AimTargetXDutyCycle())+
//   (Math.sin(getYaw())*AimTargetZDutyCycle()),-.8,.8);
//   return 
//    target;
// }



// }
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

public class limelight extends SubsystemBase {
  // NetworkTables
  private final NetworkTable table;
  private final NetworkTableEntry tvEntry;              // target valid (0 or 1)
  private final NetworkTableEntry tidEntry;             // target id
  private final NetworkTableEntry targetPoseEntry;      // targetpose_cameraspace -> double[6]
  // internal state
  private double[] targetPose = new double[6]; // always length 6 for safety
  private int targetId = -1;
  private boolean isTargeting = false;
  private final Timer timer = new Timer();

  // tuning / offsets
  private static final double DEFAULT_SIDE_OFFSET = 0.1; // meters offset used for left/right
  private static final double DUTY_CLAMP = 0.8;

  public limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tvEntry = table.getEntry("tv");
    tidEntry = table.getEntry("tid");
    targetPoseEntry = table.getEntry("targetpose_cameraspace");

    // initialize with safe defaults
    for (int i = 0; i < targetPose.length; i++) {
      targetPose[i] = 0.0;
    }

    // Example of setting filters: make sure you call this with the correct signatures
    // int[] validIDs = {3, 4};
    // LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    // The call below in your original code looked incorrect: check LimelightHelpers API before calling.
    // LimelightHelpers.SetRobotOrientation("limelight", 20, targetCenter, targetCenter, v, targetId, targetCenter);
  }

  @Override
  public void periodic() {
    // Read the latest values from NetworkTables each cycle
    double tv = tvEntry.getDouble(0.0); // 0.0 when no data
    // read target pose array safely
    double[] rawPose = targetPoseEntry.getDoubleArray(new double[0]);
    if (rawPose != null && rawPose.length >= 3) {
      // copy up to 6 elements, fill rest with 0
      for (int i = 0; i < targetPose.length; i++) {
        targetPose[i] = (i < rawPose.length) ? rawPose[i] : 0.0;
      }
    } else {
      // no pose available → set to zeros
      for (int i = 0; i < targetPose.length; i++) {
        targetPose[i] = 0.0;
      }
    }
    targetId = (int) tidEntry.getNumber(-1).doubleValue();
    // If tv is 1 then target valid otherwise invalid
    if (tv >= 0.5) {
      // timer usage example if you want to start a timer when a target first appears
      if (!timer.isRunning()) {
        timer.reset();
        timer.start();
      }
    } else {
      timer.stop();
    }
  }

  /* ---------------------------
   * Basic getters (safe)
   * -------------------------*/
  /** Returns true iff the limelight reports a valid target. */
  public boolean hasTarget() {
    double tv = tvEntry.getDouble(0.0);
    return tv >= 0.5;
  }

  /** X (lateral) camera-space metres (safe — returns 0 if no pose). */
  public double getX() {
    if (!hasTarget()) {
      return 0.0;
    }
    return targetPose[0];
  }

  /** Z (forward) camera-space metres (safe). */
  public double getZ() {
    if (!hasTarget()) {
      return 0.0;
    }
    return targetPose[2];
  }

  /**
   * Yaw (rotation around Y axis) — many limelight/targetpose implementations return degrees.
   * Convert to radians for robot math. If your pipeline already gives radians, remove conversion.
   */
  public double getYawRadians() {
    if (!hasTarget()) {
      return 0.0;
    }
    // guard index
    double angleDeg = targetPose.length > 4 ? targetPose[4] : 0.0;
    return Math.toRadians(angleDeg);
  }

  public int getTargetId() {
    return targetId;
  }

  /* ---------------------------
   * Convenience helpers used by your previous code
   * -------------------------*/
  public double getLeftX() {
    if (!hasTarget()) return 0.0;
    return getX() - DEFAULT_SIDE_OFFSET;
  }

  public double getRightX() {
    if (!hasTarget()) return 0.0;
    return getX() + DEFAULT_SIDE_OFFSET;
  }

  /** Example accessor that prints / returns measured X (keeps compatibility with your old methods) */
  public double getMeasureX() {
    return getX();
  }

  public double getMeasureY() {
    // If you want camera Y (vertical), check targetPose index 1
    if (!hasTarget()) return 0.0;
    return targetPose.length > 1 ? targetPose[1] : 0.0;
  }

  /* ---------------------------
   * Duty-cycle / aiming helpers (kept your logic, but made them safe)
   * -------------------------*/
  public double targetXError() {
    return getX();
  }

  public double AimTargetXDutyCycle() {
    if (!hasTarget()) return 0.0;
    double error = targetXError();
    return MathUtil.clamp(error, -DUTY_CLAMP, DUTY_CLAMP);
  }

  public double targetZError() {
    return getZ();
  }

  public double AimTargetZDutyCycle() {
    if (!hasTarget()) return 0.0;
    double val = MathUtil.clamp(targetZError(), -DUTY_CLAMP, DUTY_CLAMP);
    return val;
  }

  public double targetYawError() {
    // depending on how you want sign, you used -getYaw() before
    return -getYawRadians();
  }

  public double AimTargetYawDutyCycle() {
    if (!hasTarget()) return 0.0;
    double target = MathUtil.clamp(targetYawError() * 1.05, -DUTY_CLAMP, DUTY_CLAMP);
    return target;
  }

  public double RobotXDutyCycle() {
    if (!hasTarget()) return 0.0;
    double yaw = getYawRadians();
    double xDuty = AimTargetXDutyCycle();
    double zDuty = AimTargetZDutyCycle();
    double target = (-Math.sin(yaw) * xDuty) + (Math.cos(yaw) * zDuty);
    return MathUtil.clamp(target, -DUTY_CLAMP, DUTY_CLAMP);
  }

  public double RobotYDutyCycle() {
    if (!hasTarget()) return 0.0;
    double yaw = getYawRadians();
    double xDuty = AimTargetXDutyCycle();
    double zDuty = AimTargetZDutyCycle();
    double target = (Math.cos(yaw) * xDuty) + (Math.sin(yaw) * zDuty);
    return MathUtil.clamp(target, -DUTY_CLAMP, DUTY_CLAMP);
  }

  /* ---------------------------
   * Optional: targeting state toggles
   * -------------------------*/
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

