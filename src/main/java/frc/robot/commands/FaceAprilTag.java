package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import main.java.frc.robot.subsystems.Limelight2;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/**
 * Faces the currently seen AprilTag by driving rotation to reduce Limelight tx to zero.
 * Uses a simple proportional open-loop rotation; translation is held at zero so you only spin.
 */
public class FaceAprilTag extends Command {

  private final CommandSwerveDrivetrain myDrivetrain;
  private final double myTargetAngle;
  private final PIDController yawPID;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
  double yawError;
  boolean end = false;


  public FaceAprilTag(CommandSwerveDrivetrain drivetrain, double targetAngle) {
    myDrivetrain = drivetrain;
    myTargetAngle = targetAngle;
    yawPID = new PIDController(1.2, 0, 0.15);
    yawPID.enableContinuousInput(-180.0, 180.0);
    yawPID.setTolerance(2.0);
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yawPID.setSetpoint(myTargetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentYaw = Units.radiansToDegrees(Limelight2.getRobotYaw());
    double omega = yawPID.calculate(currentYaw);
    System.out.println("pre-clamp omega "+omega);
    omega = Units.degreesToRadians(omega);
    omega = MathUtil.clamp(omega, -Math.PI, Math.PI);
    yawError = yawPID.getError();
    System.out.println("Yaw error: " + yawError);
    System.out.println("current omega " +omega);
    System.out.println("Yaw deg: " + currentYaw);

    if(yawError < 10.0){
      omega = 0;
      end = true;
    }

    myDrivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(omega));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myDrivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    end = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(yawPID.atSetpoint()){
      end = true;
    }
    return end;
  }
}
