// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.cArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cUp extends Command {
  /** Creates a new cUp. */
  cArm colArm;
  public cUp(cArm coralArm) {

    // Use addRequirements() here to declare subsystem dependencies.
      colArm = coralArm;
      addRequirements(colArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colArm.setcArm(8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colArm.stopcArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
