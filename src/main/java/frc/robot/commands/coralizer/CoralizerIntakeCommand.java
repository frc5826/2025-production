package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

public class CoralizerIntakeCommand extends LoggedCommand {

    private CoralizerSubsystem coralizerSubsystem;
    private IntakeDirection direction;
    private double timer;

    public CoralizerIntakeCommand(CoralizerSubsystem coralizerSubsystem, IntakeDirection direction) {
        this.coralizerSubsystem = coralizerSubsystem;
        this.direction = direction;
        addRequirements(coralizerSubsystem);
        timer = 0;
    }

    @Override
    public void initialize() {
        switch (direction){
            case IN -> coralizerSubsystem.setIntakeSpeed(1);
            case OUT -> coralizerSubsystem.setIntakeSpeed(-0.4);
            case SHOOT -> coralizerSubsystem.setIntakeSpeed(-1);
        }
        timer = 0.25;
    }

    @Override
    public void execute() {
        if((direction != IntakeDirection.OUT && coralizerSubsystem.hasCoral()) || (direction == IntakeDirection.OUT && !coralizerSubsystem.hasCoral())) {timer -= 0.02;}
    }

    @Override
    public boolean isFinished() {
        if(timer <= 0){
            if (direction != IntakeDirection.OUT) {
                return coralizerSubsystem.hasCoral();
            }
            return !coralizerSubsystem.hasCoral();
        }
        return false;
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
