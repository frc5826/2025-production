package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

public class DealgCoralizerCommand extends LoggedCommand {

    CoralizerSubsystem c;

    public DealgCoralizerCommand(CoralizerSubsystem coralizerSubsystem) {
        this.c = coralizerSubsystem;
    }

    @Override
    public void initialize() {
        super.initialize();
        c.setIntakeSpeed(0.35);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        c.setIntakeSpeed(-0.05);
    }
}
