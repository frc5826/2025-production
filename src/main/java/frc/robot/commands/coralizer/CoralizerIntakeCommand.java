package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

public class CoralizerIntakeCommand extends LoggedCommand {

    private CoralizerSubsystem coralizerSubsystem;
    private IntakeDirection direction;

    public CoralizerIntakeCommand(CoralizerSubsystem coralizerSubsystem, IntakeDirection direction) {
        this.coralizerSubsystem = coralizerSubsystem;
        this.direction = direction;
        addRequirements(coralizerSubsystem);
    }

    @Override
    public void initialize() {
        switch (direction){
            case IN -> coralizerSubsystem.setIntakeSpeed(1);
            case OUT -> coralizerSubsystem.setIntakeSpeed(-0.15);
            case SHOOT -> coralizerSubsystem.setIntakeSpeed(-1);
        }
    }

    @Override
    public boolean isFinished() {
        if(direction != IntakeDirection.OUT) {return coralizerSubsystem.hasCoral();}
        return !coralizerSubsystem.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        coralizerSubsystem.setIntakeSpeed(0);
    }

    public static enum IntakeDirection{
        IN,
        OUT,
        SHOOT
    }

}
