// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class toL4 extends Command {
  /** Creates a new toL4. */
   Elevator el;
  public toL4(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    el = elevator;
    addRequirements(el);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // el.setPoint(225);
    el.turnPoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    el.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return el.setPoint(227);
    // return true;
  }
}
