package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Faces the currently seen AprilTag by driving rotation to zero tx.
 * Uses an open-loop rotational rate proportional to tx; translation is zeroed.
 */
public class FaceAprilTag extends Command {
  private static final double kP = 0.04; // adjust as needed
  private static final double kMaxRotRadPerSec = Math.toRadians(2.5);

  private final CommandSwerveDrivetrain drivetrain;
  private final Limelight limelight;

  private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();

  public FaceAprilTag(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    addRequirements(drivetrain, limelight);
  }

  @Override
  public void execute() {
    if (!limelight.hasTarget()) {
      drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      return;
    }
    double errorDeg = limelight.getTx();
    double outputRadPerSec = kP * Math.toRadians(errorDeg);
    outputRadPerSec = Math.max(-kMaxRotRadPerSec, Math.min(kMaxRotRadPerSec, outputRadPerSec));
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(outputRadPerSec));
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }
}
