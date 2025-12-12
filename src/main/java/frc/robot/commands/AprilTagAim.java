// // package frc.robot.commands;
// // import frc.robot.subsystems.CommandSwerveDrivetrain;
// // import frc.robot.subsystems.limelight;

// // import com.ctre.phoenix6.swerve.SwerveRequest;

// // import edu.wpi.first.wpilibj.Timer;
// // import edu.wpi.first.wpilibj2.command.Command;
// // import static edu.wpi.first.units.Units.*;

// // public class AprilTagAim extends Command {
// //   private final limelight limee;
// //   private final CommandSwerveDrivetrain drivetrainie;
// //   Timer timer;
// //   private final double stopDistanceMeters = 0.70;  
// //   private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
// //   public AprilTagAim(limelight lime, CommandSwerveDrivetrain drivetrain) {
// //     // Use addRequirements() here to declare subsystem dependencies.
   
// //    limee = lime;
// //     drivetrainie = drivetrain;
// //     timer = new Timer();
// //     // MZ addRequirements(limee, drivetrainie); 
// //     addRequirements(drivetrainie);
// //   }

// //   // Called when the command is initially scheduled.
// //   @Override
// //   public void initialize() {
// //     timer.reset();//MZ new
// //     timer.start();
// //   }

// //   // Called every time the scheduler runs while the command is scheduled.
// //   @Override
// //   public void execute() {
// // //     if(!limee.hasTarget()){
// // //         timer.reset();
// // //     }

// // //         drivetrainie.applyRequest(() ->
// // //     new SwerveRequest.FieldCentric().withVelocityX(limee.RobotXDutyCycle() * 3) // Drive forward with negative Y (forward)
   
// // //     .withVelocityY(limee.RobotYDutyCycle()* 3) // Drive left with negative X (left)
// // //         .withRotationalRate(
          
// // //         limee.AimTargetYawDutyCycle()
          
// // //           * MaxAngularRate) // Drive counterclockwise with negative X (left)
// // // ).execute();
// // if (!limee.hasTarget()) {
// //   drivetrainie.applyRequest(() ->
// //       new SwerveRequest.FieldCentric()
// //           .withVelocityX(0)
// //           .withVelocityY(0)
// //           .withRotationalRate(0)
// //   ).execute();
// //   return;
// // }

// // timer.reset(); // reset timeout while valid target exists

// // double distance = limee.getZ();
// // double slowFactor = Math.min(1.0, distance / stopDistanceMeters);

// // drivetrainie.applyRequest(() ->
// //     new SwerveRequest.FieldCentric()
// //         .withVelocityX(limee.RobotXDutyCycle() * 3 * slowFactor)
// //         .withVelocityY(limee.RobotYDutyCycle() * 3)
// //         .withRotationalRate(limee.AimTargetYawDutyCycle() * MaxAngularRate)
// // ).execute();
// // }
// //   }
  

// //   // Called once the command ends or is interrupted.
// //   @Override
// //   public void end(boolean interrupted) {
// //     drivetrainie.applyRequest(() ->
// //     new SwerveRequest.FieldCentric().withVelocityX(0) // Drive forward with negative Y (forward)
// //     .withVelocityY(0) // Drive left with negative X (left)
// //     .withRotationalRate(0) // Drive counterclockwise with negative X (left)
// // ).execute();
// //   }

// //   // Returns true when the command should end.
// //   @Override
// //   public boolean isFinished() {
// //     return 
// //     (Math.abs(limee.targetXError()) < 0.0041 //0.05, 0.0041
// //     && Math.abs(limee.targetZError()) < 0.015 //0.108, 0.015
// //     && Math.abs(limee.targetYawError()) < 0.025//0.05, 0.025
// //     ) || timer.hasElapsed(1);
// //   }
// // }


// package frc.robot.commands;

// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.limelight;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;

// import static edu.wpi.first.units.Units.*;

// public class AprilTagAim extends Command {

//   private final limelight limee;
//   private final CommandSwerveDrivetrain drivetrainie;
//   private final Timer timer = new Timer();

//   private final double MaxAngularRate =
//       RotationsPerSecond.of(0.75).in(RadiansPerSecond);

//   private final double stopDistanceMeters = 0.1;  // <<< set target distance

//   public AprilTagAim(limelight lime, CommandSwerveDrivetrain drivetrain) {
//     limee = lime;
//     drivetrainie = drivetrain;
//     addRequirements(drivetrainie);
//   }

//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//   }

//   @Override
//   public void execute() {

//     // Safety: if no target, do not keep driving
//     if (!limee.hasTarget()) {
//       drivetrainie.applyRequest(() ->
//           new SwerveRequest.FieldCentric()
//               .withVelocityX(0)
//               .withVelocityY(0)
//               .withRotationalRate(0)
//       ).execute();
//       return;
//     }

//     timer.reset(); // reset timeout while valid target exists

//     double distance = limee.getZ();
//     double slowFactor = Math.min(1.0, distance / stopDistanceMeters);

//     drivetrainie.applyRequest(() ->
//         new SwerveRequest.FieldCentric()
//             .withVelocityX(limee.RobotXDutyCycle() * 3 * slowFactor)
//             .withVelocityY(limee.RobotYDutyCycle() * 3)
//             .withRotationalRate(limee.AimTargetYawDutyCycle() * MaxAngularRate)
//     ).execute();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drivetrainie.applyRequest(() ->
//         new SwerveRequest.FieldCentric()
//             .withVelocityX(0)
//             .withVelocityY(0)
//             .withRotationalRate(0)
//     ).execute();
//   }

//   @Override
//   public boolean isFinished() {

//     // No target? Safety timeout.
//     if (!limee.hasTarget())
//       return timer.hasElapsed(1);

//     // Stop when distance reached
//     if (limee.getZ() <= stopDistanceMeters)
//       return true;

//     // Optional finish when fully aligned
//     boolean aligned =
//         Math.abs(limee.targetXError()) < 0.005 &&
//         Math.abs(limee.targetYawError()) < 0.03;

//     return aligned || timer.hasElapsed(2);
//   }
// }
package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.*;

public class AprilTagAim extends Command {

  private final limelight limee;
  private final CommandSwerveDrivetrain drivetrainie;
  private final Timer timer = new Timer();

  // Max rotation rate
  private final double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // <<< EXACT 1 FOOT (0.3048 meters)
  private final double stopDistanceMeters = 0.3048;

  public AprilTagAim(limelight lime, CommandSwerveDrivetrain drivetrain) {
    limee = lime;
    drivetrainie = drivetrain;
    addRequirements(drivetrainie);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {

    // If no target, STOP SAFELY
    if (!limee.hasTarget()) {
      drivetrainie.applyRequest(() ->
          new SwerveRequest.FieldCentric()
              .withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(0)
      ).execute();
      return;
    }

    // Reset timeout while target exists
    timer.reset();

    // Distance from camera to tag (forward/back)
    double distance = limee.getZ();

    // Slow down as you approach the 1-foot mark
    double slowFactor = Math.min(1.0, (distance - stopDistanceMeters) / 0.5);
    slowFactor = Math.max(slowFactor, 0.0); // never negative

    drivetrainie.applyRequest(() ->
        new SwerveRequest.FieldCentric()
            .withVelocityX(limee.RobotXDutyCycle() * 3 * 0.5) // forward/back
            .withVelocityY(limee.RobotYDutyCycle() * 3)              // strafe
            .withRotationalRate(limee.AimTargetYawDutyCycle() * MaxAngularRate)
    ).execute();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainie.applyRequest(() ->
        new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
    ).execute();
  }

  @Override
  public boolean isFinished() {

    // Failsafe timeout
    if (!limee.hasTarget())
      return timer.hasElapsed(1.0);

    // Stop exactly at 1 foot
    if (limee.getZ() <= stopDistanceMeters)
      return true;

    // Optional: Stop when fully aligned
    boolean aligned =
        Math.abs(limee.targetXError()) < 0.01 &&      // sideways aligned
        Math.abs(limee.targetYawError()) < 0.04;      // rotation aligned

    return aligned || timer.hasElapsed(2.0);
  }
}
