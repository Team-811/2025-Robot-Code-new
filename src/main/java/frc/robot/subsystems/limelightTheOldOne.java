
// package main.java.frc.robot.subsystems;
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
// import edu.wpi.first.math.geometry.Pose2d;

// import java.util.ArrayList;
// import org.opencv.core.Mat;
// import org.opencv.core.Point;
// import org.opencv.core.Scalar;
// import org.opencv.imgproc.Imgproc;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.math.MathUtil;


// public class limelightTheOldOne extends SubsystemBase {
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


//   public limelightTheOldOne() {
//     // var visionThread = new Thread(this::apriltagVisionThreadProc);
//     // visionThread.setDaemon(true);
//     // visionThread.start();
//     timer = new Timer();
//     positionX = myPose.getMeasureX();
//     positionY = myPose.getMeasureY();
    
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
