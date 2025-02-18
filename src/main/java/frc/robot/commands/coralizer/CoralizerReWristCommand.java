package frc.robot.commands.coralizer;

import frc.robot.subsystems.CoralizerSubsystem;

public class CoralizerReWristCommand extends CoralizerWristCommand {

    private double rotationOffset;

    public CoralizerReWristCommand(CoralizerSubsystem coralizerSubsystem, double rotationOffset) {
        super(coralizerSubsystem, 0);
        this.rotationOffset = rotationOffset;
    }

    @Override
    public void initialize() {
        double targetRotation = coralizerSubsystem.getWristTarget() + rotationOffset;
        super.initialize();
        coralizerSubsystem.setWristTarget(targetRotation);
    }
}
