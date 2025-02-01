package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

public class CoralizerWristCommand extends LoggedCommand {

    private CoralizerSubsystem coralizerSubsystem;
    private double rotation;

    public CoralizerWristCommand(CoralizerSubsystem coralizerSubsystem, double rotation) {
        this.coralizerSubsystem = coralizerSubsystem;
        this.rotation = rotation;
    }

    @Override
    public void initialize() {
        coralizerSubsystem.setWristTarget(rotation);
    }
}
