// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.cArm;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.rollerClaw;
import static edu.wpi.first.units.Units.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cL4x2 extends ParallelCommandGroup {
  /** Creates a new cL4x2. */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)* OperatorConstants.kSpeed; // kSpeedAt12Volts desired top speed
            private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
     private final RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed*0.05).withRotationalDeadband(MaxAngularRate*0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
          private limelight lime = new limelight();
            private SlewRateLimiter slewLimY = new SlewRateLimiter(1.5);
          private SlewRateLimiter slewLimX = new SlewRateLimiter(1.5);
          private SlewRateLimiter slewLimRoteLime = new SlewRateLimiter(1.5);
  public cL4x2(Elevator el, CommandSwerveDrivetrain drivetrain, rollerClaw rolly, String path,cArm coArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
         addCommands(new PathPlannerAuto("midL4x1"),new ParallelDeadlineGroup(new WaitCommand(2
     ),drivetrain.applyRequest(()->robotCentric
     .withVelocityX(slewLimY.calculate(limeY())*MaxSpeed)
     .withVelocityY(slewLimX.calculate(limeX())*MaxSpeed)
     .withRotationalRate(slewLimRoteLime.calculate(limeYaw())/60)
     ).ignoringDisable(true),new cUpAuto(coArm)), new slowRolly(rolly), new PathPlannerAuto("toSourceAuto"), new ParallelDeadlineGroup(new WaitCommand(2), drivetrain.applyRequest(()->robotCentric
     .withVelocityX(slewLimY.calculate(limeY())*MaxSpeed)
     .withVelocityY(slewLimX.calculate(limeX())*MaxSpeed)
     .withRotationalRate(slewLimRoteLime.calculate(limeYaw())/60)
     ).ignoringDisable(true),new cMid(coArm)), new reverseIntakeCommand(rolly), new PathPlannerAuto("fromSource"), new ParallelDeadlineGroup(new WaitCommand(2), drivetrain.applyRequest(()->robotCentric
     .withVelocityX(slewLimY.calculate(limeY())*MaxSpeed)
     .withVelocityY(slewLimX.calculate(limeX())*MaxSpeed)
     .withRotationalRate(slewLimRoteLime.calculate(limeYaw())/60)
     ).ignoringDisable(true),new cUpAuto(coArm)), new slowRolly(rolly));

  //make a new cMidAuto
  }
  public double limeX(){
    double limeLeftX = lime.getX();
    return limeLeftX;
  }
  public double limeX_Left(){
    double limeXLeft = lime.getLeftX();
    return limeXLeft;
  }
  public double limeX_Right(){
    double limeXRight = lime.getRightX();
    return limeXRight;
  }
  public double limeYaw(){
    double limeRightx = lime.getYaw();
    return limeRightx;
  }
  public double limeY(){
    double limeleftY = lime.getV()*0.5;
    return limeleftY;
  }
}
