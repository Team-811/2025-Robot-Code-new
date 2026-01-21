// package main.java.frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Limelight;
// import main.java.frc.robot.subsystems.Limelight2;
// import main.java.frc.robot.subsystems.limelightTheOldOne;
// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
// import static edu.wpi.first.units.Units.*;
// import edu.wpi.first.math.filter.SlewRateLimiter;


// import com.ctre.phoenix6.swerve.SwerveRequest;
// public class stopMOVING extends Command {

//     limelightTheOldOne diddy;
//     private final CommandSwerveDrivetrain drivetrain;
//     private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
//     private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
//                    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)* OperatorConstants.kSpeed; // kSpeedAt12Volts desired top speed
           
//      private final RobotCentric robotCentric = new SwerveRequest.RobotCentric()
//             .withDeadband(MaxSpeed*0.05).withRotationalDeadband(MaxAngularRate*0.1)
//             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

//             private SlewRateLimiter slewLimY = new SlewRateLimiter(1.5);
//           private SlewRateLimiter slewLimX = new SlewRateLimiter(1.5);
//           private SlewRateLimiter slewLimRoteLime = new SlewRateLimiter(1.5);


//     public stopMOVING(limelightTheOldOne epstein, CommandSwerveDrivetrain drivetrain){
//         diddy = epstein;
//         this.drivetrain = drivetrain;
//         addRequirements(epstein, drivetrain);
//     }
//     @Override
//     public void initialize() {

//     }

//   // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         if(epstein.getZ()<= 2){
//             drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
//         } else{
//         //     drivetrain.applyRequest(() ->
//         //     new SwerveRequest.RobotCentric().withVelocityX(epstein.RobotXDutyCycle() * 3) // Drive forward with negative Y (forward)
           
//         //     .withVelocityY(epstein.RobotYDutyCycle()* 3) // Drive left with negative X (left)
//         //         .withRotationalRate(
                  
//         //         epstein.AimTargetYawDutyCycle()
                  
//         //           * MaxAngularRate) // Drive counterclockwise with negative X (left)
//         // ).execute();
//         // }
//         if(epstein.hasTarget()){
//             timer.reset();
//         }
    
//             drivetrainie.applyRequest(() ->
//         new SwerveRequest.RobotCentric().withVelocityX(epstein.RobotXDutyCycle() * 3) // Drive forward with negative Y (forward)
       
//         .withVelocityY(epstein.RobotYDutyCycle()* 3) // Drive left with negative X (left)
//             .withRotationalRate(
              
//             epstein.AimTargetYawDutyCycle()
              
//               * MaxAngularRate) // Drive counterclockwise with negative X (left)
//     ).execute();
//         }

//     }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
// //     drivetrain.applyRequest(() ->
// //     new SwerveRequest.RobotCentric().withVelocityX(0) // Drive forward with negative Y (forward)
// //     .withVelocityY(0) // Drive left with negative X (left)
// //     .withRotationalRate(0) // Drive counterclockwise with negative X (left)
// // ).execute();
//   }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//     return false;
//     }
// }
