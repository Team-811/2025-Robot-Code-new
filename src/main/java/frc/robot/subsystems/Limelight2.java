package main.java.frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.LimelightHelpers;



//y is up and down, z is forward and backward, x is left and right

public class Limelight2 extends SubsystemBase{
    NetworkTable table2;
    static double x, y, area, distX, distY, distZ, angleTargetRadians, v, robotYaw;
    int fiducialID;
    NetworkTableEntry tx, ty, ta, tv;
    Pose3d targetPose, botPose;
    Rotation3d targetRotation, botRotation;
    
    public Limelight2(){
        table2 = NetworkTableInstance.getDefault().getTable("limelight-lime");
        tx = table2.getEntry("tx");
        ty = table2.getEntry("ty");
        ta = table2.getEntry("ta");
        tv = table2.getEntry("tv");
    }

    public void updateValues(){
        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        v = tv.getDouble(0.0);

        targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-lime");
        distX = targetPose.getX();
        distY = targetPose.getY();
        distZ = targetPose.getZ();

        botPose = LimelightHelpers.getBotPose3d_wpiBlue("limelight-lime");
        botRotation = botPose.getRotation();
        robotYaw = botRotation.getZ();

        targetRotation = targetPose.getRotation();
        angleTargetRadians = targetRotation.getZ();

        fiducialID = (int)LimelightHelpers.getFiducialID("limelight-lime");
    }
    public void updateDashboard(){
        //post to smart dashboard periodically
        SmartDashboard.putNumber("Limelight2XDegrees", x);
        SmartDashboard.putNumber("Limelight2YDegrees", y);
        SmartDashboard.putNumber("Limelight2YDegrees", v);
        SmartDashboard.putNumber("Limelight2Area", area);
        SmartDashboard.putNumber("Limelight2DistanceX", distX);
        SmartDashboard.putNumber("Limelight2DistanceY", distY);
        SmartDashboard.putNumber("Limelight2DistanceZ", distZ);
        SmartDashboard.putNumber("Limelight2TargetYawRadians", angleTargetRadians);
    }
    @Override
    public void periodic(){
        updateValues();
        updateDashboard();
    }

    public static double getDistZ(){
        return distZ;
    }

    public static double getAngleTargetRadians(){
        return angleTargetRadians;
    }

    public static double getAngleTargetDegrees(){
        return Units.radiansToDegrees(angleTargetRadians);
    }

    public static double getRobotYaw(){
        return robotYaw;
    }

    public static double getTV(){
        return v;
    }
}
