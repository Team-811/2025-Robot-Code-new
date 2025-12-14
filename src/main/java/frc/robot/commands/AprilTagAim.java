
package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.*;

public class AprilTagAim extends Command {

  private final Limelight limee;
  private final CommandSwerveDrivetrain drivetrainie;
  private final Timer timer = new Timer();

  // Max rotation rate
  private final double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // <<< EXACT 1 FOOT (0.3048 meters)
  private final double stopDistanceMeters = 0.3048;

  public AprilTagAim(Limelight lime, CommandSwerveDrivetrain drivetrain) {
    limee = lime;
    drivetrainie = drivetrain;
    addRequirements(drivetrainie);
  }

  @Override
  public void initialize() {
    limee.setTargeting(true);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {

    // If no target, STOP SAFELY
    if (!limee.hasTarget()) {
      timer.stop();
      drivetrainie.applyRequest(() ->
          new SwerveRequest.FieldCentric()
              .withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(0)
      ).execute();
      return;
    }

    // Restart timeout only when we have a target after being lost
    if (!timer.isRunning()) {
      timer.reset();
      timer.start();
    }

    // Distance from camera to tag (forward/back)
    double distance = limee.getZ();

    // Slow down as you approach the 1-foot mark
    final double slowFactor = Math.max(Math.min(1.0, (distance - stopDistanceMeters) / 0.5), 0.0); // clamp 0-1

    drivetrainie.applyRequest(() ->
        new SwerveRequest.FieldCentric()
            .withVelocityX(limee.RobotXDutyCycle() * 3 * slowFactor) // forward/back with ramp
            .withVelocityY(limee.RobotYDutyCycle() * 3 * slowFactor) // strafe with ramp
            .withRotationalRate(limee.AimTargetYawDutyCycle() * MaxAngularRate)
    ).execute();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    limee.setTargeting(false);
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

