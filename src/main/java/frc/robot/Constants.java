// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOpControllerPort = 1;
    public static final double kJoyRightXDeadzone = 0.5;
    public static final double kJoyLeftXDeadzone = 0.5;
    public static final double kJoyLeftYDeadzone = 0.5;
    public static final double kSpeed = 0.8;//number between 0-1 muliplies the max speed
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
    public static final int climbId = 32;

    public static final double fastSpeed =1 ;
    public static final double slowSpeed = 0.5;
    public static final double normalSpeed = 0.9;
    public static final double kElDeadBand = 3;
  
  }
}
