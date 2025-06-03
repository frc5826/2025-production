package frc.robot.commands.coralizer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class HoldAlgaeCommand extends Command {

    ShooterSubsystem sh;

    public HoldAlgaeCommand(ShooterSubsystem shooterSubsystem) {
        this.sh = shooterSubsystem;
        addRequirements(sh);
    }

    @Override
    public void initialize() {
        super.initialize();
        sh.setIntakeSpeed(0.9);
    }
}
