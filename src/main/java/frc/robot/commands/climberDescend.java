// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.climber;
// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class climberDescend extends Command {
//   /** Creates a new climberDescend. */
//   climber climb;
//   public climberDescend(climber climber) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     climb =climber;
//     addRequirements(climb);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     climb.setThePoint(10);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   //  climb.descend();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     climb.stopClimb();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
