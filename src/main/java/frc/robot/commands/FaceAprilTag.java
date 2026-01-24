// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import main.java.frc.robot.subsystems.Limelight2;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.math.MathUtil;

// /**
//  * Faces the currently seen AprilTag by driving rotation to reduce Limelight tx to zero.
//  * Uses a simple proportional open-loop rotation; translation is held at zero so you only spin.
//  */
// public class FaceAprilTag extends Command {
//   private static final double kP = 0.04; // adjust as needed
//   private static final double kMaxRotRadPerSec = Math.toRadians(2.5);

//   private final CommandSwerveDrivetrain drivetrain;
//   private final Limelight2 lime;

//   private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
//   double radPerS = 0.0;

//   public FaceAprilTag(CommandSwerveDrivetrain drive, Limelight2 limelight) {
//     drivetrain = drive;
//     lime = limelight;
//     addRequirements(drivetrain, lime);
//   }
//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
   
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
