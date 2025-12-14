package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class limeStartTarget extends Command {

    private final Limelight lime;
    private final CommandSwerveDrivetrain drivetrain;

    public limeStartTarget(Limelight lime, CommandSwerveDrivetrain drivetrain) {
        this.lime = lime;
        this.drivetrain = drivetrain;
        addRequirements(lime);
    }

    @Override
    public void initialize() {

        // If Limelight already sees a target  start alignment immediately
        if (lime.hasTarget()) {
            new AprilTagAim(lime, drivetrain).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // end immediately, the launched command keeps running
    }
}

