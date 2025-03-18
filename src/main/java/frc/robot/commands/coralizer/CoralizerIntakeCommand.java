package frc.robot.commands.coralizer;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

public class CoralizerIntakeCommand extends LoggedCommand {

    private CoralizerSubsystem coralizerSubsystem;
    private IntakeDirection direction;
    private Timer releaseTimer;
    private Timer expirationTimer;

    public CoralizerIntakeCommand(CoralizerSubsystem coralizerSubsystem, IntakeDirection direction) {
        this.coralizerSubsystem = coralizerSubsystem;
        this.direction = direction;
        addRequirements(coralizerSubsystem);

        releaseTimer = new Timer();
        expirationTimer = new Timer();
    }

    @Override
    public void initialize() {
        super.initialize();
        coralizerSubsystem.setShouldHold(false);
        switch (direction){
            case IN -> coralizerSubsystem.setIntakeSpeed(-0.6);
            case OUT -> coralizerSubsystem.setIntakeSpeed(0.8);
            case SHOOT -> coralizerSubsystem.setIntakeSpeed(1);
        }
        releaseTimer.reset();
        expirationTimer.reset();
        expirationTimer.start();
    }

    @Override
    public void execute() {
        if((direction == IntakeDirection.IN && coralizerSubsystem.hasCoral()) || (direction != IntakeDirection.IN && !coralizerSubsystem.hasCoral())) {releaseTimer.start();}
    }

    @Override
    public boolean isFinished() {
        if(releaseTimer.get() >= 0.25 && direction == IntakeDirection.IN){
            return coralizerSubsystem.hasCoral();
        } else if (releaseTimer.get() >= 0.6 && direction == IntakeDirection.OUT) {
            return !coralizerSubsystem.hasCoral();
        }
        return false;

//        if (expirationTimer.get() >= 7) {return true;}
//        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        coralizerSubsystem.setIntakeSpeed(-0.05);
        expirationTimer.reset();
        releaseTimer.reset();
    }

    public static enum IntakeDirection{
        IN,
        OUT,
        SHOOT
    }

}
