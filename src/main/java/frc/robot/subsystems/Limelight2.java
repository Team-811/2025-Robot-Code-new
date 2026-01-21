package main.java.frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.Units;

import frc.robot.LimelightHelpers;



//y is up and down, z is forward and backward, x is left and right

public class Limelight2 extends SubsystemBase{
    NetworkTable table2;
    double x, y, area, distX, distY,distZ;
    NetworkTableEntry tx, ty, ta;
    Pose3d targetPose, botPose;
    Rotation3d botRotation, targetRotation;
    
    public Limelight2(){
        table2 = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table2.getEntry("tx");
        ty = table2.getEntry("ty");
        ta = table2.getEntry("ta");

    }

    public void updateValues(){
        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");

        distX = targetPose.getX();
        distY = targetPose.getY();
        distZ = targetPose.getZ();

    }
    // public double AimTargetXDutyCycle(){
    //     if (! hasTarget()) {
    //       return 0;
    //     }
    //     double target;
    //     double Error =  targetXError();
    //       target= MathUtil.clamp(
    //    (
    //       Error
      
    //     ),-0.8,0.8); 
    //    return target;
       
    //    }
    //    public double RobotXDutyCycle(){
    //     if (!hasTarget()) return 0;
    //    double target = MathUtil.clamp((-Math.sin(getYaw())*AimTargetXDutyCycle())+(Math.cos(getYaw())
       
    //    *AimTargetZDutyCycle()),-.8,.8);
      
    //    return target;
    //   }
    //   public boolean hasTarget(){
    //     return tv.getDouble(0) ==1;
    //   }
    //   public double getYaw(){
    //     // System.out.println(targetPose[4]);
    //     // return targetPose[4]*v;
    //     return targetPose[4] * Math.PI/180;
    //   }
// public double limeX(){
//             double limeLeftX = lime.getX();
//             return limeLeftX;
//           }
//             public double limeYaw(){
//     double limeRightx = lime.getYaw();
//     return limeRightx;
//   }
//   public double limeY(){
//     double limeleftY = lime.getY()*0.5;
//     return limeleftY;
//   }

    public void updateDashboard(){
        //post to smart dashboard periodically
        SmartDashboard.putNumber("Limelight2X", x);
        SmartDashboard.putNumber("Limelight2Y", y);
        SmartDashboard.putNumber("Limelight2Area", area);
        SmartDashboard.putNumber("Limelight2DistanceX", distX);
        SmartDashboard.putNumber("Limelight2DistanceY", distY);
        SmartDashboard.putNumber("Limelight2DistanceZ", distZ);
    }
    @Override
    public void periodic(){
        updateValues();
        updateDashboard();
    }
}
