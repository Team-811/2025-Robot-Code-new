// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
// Controller Guide - Driver Xbox
//   Left stick: strafe (field-centric X/Y)
//   Right stick X: rotate; deadbands 0.1
//
// Speed modes (Drive/SpeedMode on dashboard):
//   Right bumper = Slow (0.5)
//   Left bumper  = Fast (1.0)
//   Neither      = Normal (0.8)
//
// Limelight (hold-to-run):
//   Right trigger: CenterToTagOneMeter (drive to ~1 m, center X and yaw)
//   Left trigger:  AprilTagAim (drive/strafe/yaw to ~0.3048 m, slow near goal)
//   Pipeline/LEDs must be set to AprilTag with LEDs on; no target -> zero output.
//
// Start button: reseed field-centric heading.
// Start + Y (held): SysId quasistatic forward on drivetrain.
// Start + X (held): SysId quasistatic reverse on drivetrain.
// 
// Notes:
// Default command is field-centric drive; MaxSpeed is scaled by kSpeed=1.0.
// SysId bindings require robot in a safe state; they override normal driving while held.
// Limelight: Targeting is enabled/disabled inside the aim commands; 
//This file is loacted C:\Users\Team 811\FRC\2025-Robot-Code-new\2025-Robot-Code-new\src\main\java\frc\robot
//            Ensure the tag pipeline is active and LEDs on when using triggers.
package frc.robot;

/*
 * File Overview: Central wiring hub for subsystems, commands, and driver controls.
 * Features/Details:
 * - Creates drivetrain (CTRE swerve), Limelight, telemetry logger, and autonomous chooser.
 * - Defines driver Xbox bindings: field-centric default drive, speed modes via bumpers, vision assists on triggers, SysId on start+X/Y.
 * - Applies joystick deadbands/slew rate limiting and speed scaling for smooth control.
 * - Publishes driver-facing telemetry (speed mode/scale, joystick values, pose, velocities) to SmartDashboard.
 * - Seeds field-centric heading at startup and registers drivetrain telemetry streaming.
 */
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
    publishStaticTelemetry();

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
    pushDriverTelemetry(mode, scale);
    return scale;
  }

  /** One-time dashboard entries that do not change at runtime. */
  private void publishStaticTelemetry() {
    SmartDashboard.putNumber("Drive/MaxSpeedMps", MaxSpeed);
    SmartDashboard.putNumber("Drive/MaxAngularRateRadPerSec", MaxAngularRate);
  }

  /** Live driver-focused telemetry for quick debugging and mode awareness. */
  private void pushDriverTelemetry(String mode, double scale) {
    SmartDashboard.putString("Drive/SpeedMode", mode);
    SmartDashboard.putNumber("Drive/SpeedScale", scale);
    SmartDashboard.putNumber("Joystick/LeftX", joyLeftX());
    SmartDashboard.putNumber("Joystick/LeftY", joyLeftY());
    SmartDashboard.putNumber("Joystick/RightX", joyRightX());

    var state = drivetrain.getState();
    if (state != null) {
      SmartDashboard.putNumber("Drive/PoseX", state.Pose.getX());
      SmartDashboard.putNumber("Drive/PoseY", state.Pose.getY());
      SmartDashboard.putNumber("Drive/HeadingDeg", state.Pose.getRotation().getDegrees());
      SmartDashboard.putNumber("Drive/MeasuredVx", state.Speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Drive/MeasuredVy", state.Speeds.vyMetersPerSecond);
      SmartDashboard.putNumber("Drive/MeasuredOmega", state.Speeds.omegaRadiansPerSecond);
    }
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







