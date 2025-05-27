package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DealgCoralizerCommand extends LoggedCommand {

    ShooterSubsystem s;

    public DealgCoralizerCommand(ShooterSubsystem shooterSubsystem) {
        this.s = shooterSubsystem;
    }

    @Override
    public void initialize() {
        super.initialize();
        s.setIntakeSpeed(0.9);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        s.setIntakeSpeed(-0.03);
    }
}
