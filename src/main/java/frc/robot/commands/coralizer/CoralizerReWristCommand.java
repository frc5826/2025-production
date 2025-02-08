package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

public class CoralizerReWristCommand extends LoggedCommand {

    private CoralizerSubsystem coralizerSubsystem;
    private double rotation;

    public CoralizerReWristCommand(CoralizerSubsystem coralizerSubsystem, double rotation) {
        this.coralizerSubsystem = coralizerSubsystem;
        this.rotation = rotation;
    }

    @Override
    public void initialize() {
        coralizerSubsystem.setWristTarget(rotation + coralizerSubsystem.getWristTarget());
    }

    @Override
    public boolean isFinished() {
        return true;//Math.abs(coralizerSubsystem.getRotation() - rotation) <= cCoralizerDeadband;
    }
}
