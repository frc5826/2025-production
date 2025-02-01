package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

public class CoralizerIntakeCommand extends LoggedCommand {

    private CoralizerSubsystem coralizerSubsystem;
    private double speed;

    public CoralizerIntakeCommand(CoralizerSubsystem coralizerSubsystem, double speed) {
        this.coralizerSubsystem = coralizerSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        coralizerSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        coralizerSubsystem.setIntakeSpeed(0);
    }
}
