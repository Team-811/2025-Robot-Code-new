// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Limelight;

// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.RotationsPerSecond;

// /**
//  * Drives the robot to center on an AprilTag and stop 1 meter away while facing it.
//  * Runs until the pose error is small or until the target is lost.
//  */
// public class CenterToTagOneMeter extends Command {

//   private static final double TARGET_DISTANCE_METERS = 1.0;
//   private static final double KP_TRANSLATION = 1.5; // scale translational error to m/s
//   private static final double KP_YAW = 2.5;         // scale yaw error to rad/s
//   private static final double MAX_TRANS_SPEED = 1.5; // m/s clamp
//   private static final double MAX_YAW_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
//   private static final double LOSS_DEBOUNCE_S = 0.2; // avoid stutter on brief loss

//   private final Limelight lime;
//   private final CommandSwerveDrivetrain drivetrain;
//   private final Timer lossTimer = new Timer();

//   public CenterToTagOneMeter(Limelight lime, CommandSwerveDrivetrain drivetrain) {
//     this.lime = lime;
//     this.drivetrain = drivetrain;
//     addRequirements(drivetrain);
//   }

//   @Override
//   public void initialize() {
//     lime.setTargeting(true);
//     lime.setLEDs(true);
//     lossTimer.stop();
//     lossTimer.reset();
//   }

//   @Override
//   public void execute() {
//     if (!lime.hasTarget() || !lime.isPoseValid()) {
//       if (!lossTimer.isRunning()) {
//         lossTimer.reset();
//         lossTimer.start();
//       }
//       if (!lossTimer.hasElapsed(LOSS_DEBOUNCE_S)) {
//         return; // keep last command outputs during brief dropouts
//       }
//       drivetrain.applyRequest(() ->
//           new SwerveRequest.FieldCentric()
//               .withVelocityX(0)
//               .withVelocityY(0)
//               .withRotationalRate(0)
//       ).execute();
//       return;
//     }
//     lossTimer.stop();
//     lossTimer.reset();

//     // Errors relative to desired pose
//     double xError = lime.getX(); // strafe to zero
//     double distanceError = lime.getZ() - TARGET_DISTANCE_METERS; // positive = too far
//     double yawError = lime.getYawRadians(); // positive = rotate CCW to face tag

//     // Simple P control with clamps
//     double vx = MathUtil.clamp(-distanceError * KP_TRANSLATION, -MAX_TRANS_SPEED, MAX_TRANS_SPEED);
//     double vy = MathUtil.clamp(-xError * KP_TRANSLATION, -MAX_TRANS_SPEED, MAX_TRANS_SPEED);
//     double rot = MathUtil.clamp(yawError * KP_YAW, -MAX_YAW_RATE, MAX_YAW_RATE);

//     drivetrain.applyRequest(() ->
//         new SwerveRequest.FieldCentric()
//             .withVelocityX(vx)
//             .withVelocityY(vy)
//             .withRotationalRate(rot)
//     ).execute();
//   }

//   @Override
//   public void end(boolean interrupted) {
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
//     if (!lime.hasTarget() || !lime.isPoseValid()) {
//       return lossTimer.hasElapsed(LOSS_DEBOUNCE_S);
//     }
//     double xError = Math.abs(lime.getX());
//     double distanceError = Math.abs(lime.getZ() - TARGET_DISTANCE_METERS);
//     double yawError = Math.abs(lime.getYawRadians());
//     return xError < 0.05 && distanceError < 0.05 && yawError < 0.05;
//   }
// }
