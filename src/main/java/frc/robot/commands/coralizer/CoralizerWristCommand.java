package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Coralizer.*;

public class CoralizerWristCommand extends LoggedCommand {

    protected CoralizerSubsystem coralizerSubsystem;
    protected DoubleSupplier rotation;

    public CoralizerWristCommand(CoralizerSubsystem coralizerSubsystem, DoubleSupplier rotation) {
        this.coralizerSubsystem = coralizerSubsystem;
        this.rotation = rotation;
        addRequirements(coralizerSubsystem);
    }

    public CoralizerWristCommand(CoralizerSubsystem coralizerSubsystem, double rotation) {
        this(coralizerSubsystem, () -> rotation);
    }

    @Override
    public void initialize() {
        super.initialize();
        coralizerSubsystem.setWristTarget(rotation.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(coralizerSubsystem.getRotation() - rotation.getAsDouble()) <= cCoralizerDeadband;
    }
}
