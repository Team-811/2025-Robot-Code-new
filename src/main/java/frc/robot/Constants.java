// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Central place for robot-wide constants. Keep this free of behavior/logic so these values can be
 * referenced from anywhere without side effects.
 */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    private OperatorConstants() {}

    // Controller ports
    public static final int kDriverControllerPort = 0;
    public static final int kOpControllerPort = 1;

    // Shared deadzone for stick axes (override individually if needed later).
    public static final double kDefaultControllerDeadzone = 0.5;
    public static final double kJoyRightXDeadzone = kDefaultControllerDeadzone;
    public static final double kJoyLeftXDeadzone = kDefaultControllerDeadzone;
    public static final double kJoyLeftYDeadzone = kDefaultControllerDeadzone;

    // Base speed scaler applied to the drivetrain (0-1).
    public static final double kSpeed = 0.8;

    // Driver speed scaling presets
    public static final double fastSpeed = 1.0;
    public static final double slowSpeed = 0.7;
    public static final double normalSpeed = 0.7;

    // Hardware IDs and limits
    public static final double kSlipCurrent = 120;
    public static final int neoId = 18;
    public static final int elKrakenId = 23;
    public static final int fDoubSolC1 = 2;
    public static final int rDoubSolC1 = 3;
    public static final int fDoubsolCT = 6;
    public static final int rDoubSolCT = 7;
    public static final int fDoubSolA = 0;
    public static final int rDoubSolA = 1;
    public static final int cArmId = 24;
    public static final int climbId = 25;
    public static final double kElDeadBand = 3;
  }
}