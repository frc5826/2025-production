package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

import static frc.robot.Constants.Coralizer.*;

public class CoralizerWristCommand extends LoggedCommand {

    protected CoralizerSubsystem coralizerSubsystem;
    protected double rotation;

    public CoralizerWristCommand(CoralizerSubsystem coralizerSubsystem, double rotation) {
        this.coralizerSubsystem = coralizerSubsystem;
        this.rotation = rotation;
        addRequirements(coralizerSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        coralizerSubsystem.setWristTarget(rotation);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(coralizerSubsystem.getRotation() - rotation) <= cCoralizerDeadband;
    }
}
