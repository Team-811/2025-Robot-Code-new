// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AprilTagAim;
import frc.robot.commands.CenterToTagOneMeter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;
public class RobotContainer {
  private final Limelight lime = new Limelight();

  // Base speed scaling constants for the swerve (meters/sec and radians/sec).
  private final double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * OperatorConstants.kSpeed;
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // Swerve request object reused by driver bindings.
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Slew limiters tame acceleration in each axis/rotation to keep the robot smooth.
  private final SlewRateLimiter slewLimY = new SlewRateLimiter(2.0);
  private final SlewRateLimiter slewLimX = new SlewRateLimiter(2.0);
  private final SlewRateLimiter slewLimRote = new SlewRateLimiter(1.0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Driver controls (single Xbox assumed for drivetrain + vision assist).
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Seed heading at startup so field-centric drive has a sane reference.
    drivetrain.seedFieldCentric();
    configureBindings();

    // Build a PathPlanner-backed autonomous chooser and expose it to SmartDashboard.
    autoChooser = AutoBuilder.buildAutoChooser("midL4x1");
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Ex Auto", new PathPlannerAuto("Ex Auto"));
    SmartDashboard.putData("autoChooser", autoChooser);
  }

  /** Define driver -> command mappings. */
  private void configureBindings() {
    // Default command: field-centric drive with slew-limited joystick input.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive.withVelocityX(slewLimY.calculate(joyLeftY()) * MaxSpeed * speedScale())
                    .withVelocityY(slewLimX.calculate(joyLeftX()) * MaxSpeed * speedScale())
                    .withRotationalRate(slewLimRote.calculate(-joyRightX()) * MaxAngularRate)));

    // Vision-assisted align/target commands.
    driverController.rightTrigger().onTrue(new CenterToTagOneMeter(lime, drivetrain));
    driverController.leftTrigger().onTrue(new AprilTagAim(lime, drivetrain));

    // SysId bindings to characterize drivetrain when requested.
    driverController.start().and(driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Push live drivetrain telemetry to the log so you can monitor speeds, states, and odometry.
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public double joyRightX() {
    double rightX = driverController.getRightX();
    if (Math.abs(rightX) > OperatorConstants.kJoyRightXDeadzone) {
      return rightX;
    }
    return 0;
  }

  public double joyLeftX() {
    double leftX = driverController.getLeftX();
    if (Math.abs(leftX) > OperatorConstants.kJoyLeftXDeadzone) {
      return leftX;
    }
    return 0;
  }

  public double joyLeftY() {
    double leftY = driverController.getLeftY();
    if (Math.abs(leftY) > OperatorConstants.kJoyLeftYDeadzone) {
      return leftY;
    }
    return 0;
  }

  public double limeX() {
    return lime.getX();
  }

  public double limeX_Left() {
    return lime.getLeftX();
  }

  public double limeX_Right() {
    return lime.getRightX();
  }

  // Variable speed scaling based on bumper state (fast/slow/normal) to tame driver inputs.
  public double speedScale() {
    String mode = "Normal";
    double scale = Constants.OperatorConstants.normalSpeed;
    if (driverController.rightBumper().getAsBoolean()) {
      mode = "Slow";
      scale = Constants.OperatorConstants.slowSpeed;
    }
    if (driverController.leftBumper().getAsBoolean()) {
      mode = "Fast";
      scale = Constants.OperatorConstants.fastSpeed;
    }
    SmartDashboard.putString("DriveSpeedMode", mode);
    return scale;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    if (selected != null) {
      return selected;
    }
    return new InstantCommand();
  }
}







