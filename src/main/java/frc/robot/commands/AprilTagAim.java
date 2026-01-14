
// package frc.robot.commands;

// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Limelight;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;

// import static edu.wpi.first.units.Units.*;

// public class AprilTagAim extends Command {

//   private final Limelight lime;
//   private final CommandSwerveDrivetrain drivetrain;
//   private final Timer timer = new Timer();
//   private boolean lostTarget = false;

//   // Max rotation rate
//   private final double MaxAngularRate =
//       RotationsPerSecond.of(0.75).in(RadiansPerSecond);

//   // <<< EXACT 1 FOOT (0.3048 meters)
//   private final double stopDistanceMeters = 0.3048;

//   public AprilTagAim(Limelight lime, CommandSwerveDrivetrain drivetrain) {
//     this.lime = lime;
//     this.drivetrain = drivetrain;
//     addRequirements(drivetrain);
//   }

//   @Override
//   public void initialize() {
//     lime.setTargeting(true);
//     lime.setLEDs(true);
//     lostTarget = false;
//     timer.reset();
//     timer.start();
//   }

//   @Override
//   public void execute() {

//     // If no target, STOP SAFELY and finish
//     if (!lime.hasTarget() || !lime.isPoseValid()) {
//       drivetrain.applyRequest(() ->
//           new SwerveRequest.FieldCentric()
//               .withVelocityX(0)
//               .withVelocityY(0)
//               .withRotationalRate(0)
//       ).execute();
//       lostTarget = true;
//       return;
//     }

//     // Distance from camera to tag (forward/back)
//     double distance = lime.getZ();

//     // Slow down as you approach the 1-foot mark
//     final double slowFactor = Math.max(Math.min(1.0, (distance - stopDistanceMeters) / 0.5), 0.0); // clamp 0-1

//     drivetrain.applyRequest(() ->
//         new SwerveRequest.FieldCentric()
//             .withVelocityX(-lime.RobotXDutyCycle() * 3 * slowFactor) // invert to match field-centric forward
//             .withVelocityY(-lime.RobotYDutyCycle() * 3 * slowFactor) // invert to match field-centric left
//             .withRotationalRate(lime.AimTargetYawDutyCycle() * MaxAngularRate)
//     ).execute();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     timer.stop();
//     lime.setTargeting(false);
//     lime.setLEDs(false);
//     drivetrain.applyRequest(() ->
//         new SwerveRequest.FieldCentric()
//             .withVelocityX(0)
//             .withVelocityY(0)
//             .withRotationalRate(0)
//     ).execute();
//   }

//   @Override
//   public boolean isFinished() {

//     // Failsafe timeout
//     if (lostTarget)
//       return true;

//     // Stop exactly at 1 foot
//     if (lime.getZ() <= stopDistanceMeters)
//       return true;

//     // Optional: Stop when fully aligned
//     boolean aligned =
//         Math.abs(lime.targetXError()) < 0.01 &&      // sideways aligned
//         Math.abs(lime.targetYawError()) < 0.04;      // rotation aligned

//     return aligned || timer.hasElapsed(2.0);
//   }
// }

