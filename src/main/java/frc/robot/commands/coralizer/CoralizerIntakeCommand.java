package frc.robot.commands.coralizer;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CoralizerIntakeCommand extends LoggedCommand {

    private ShooterSubsystem shooterSubsystem;
    private IntakeDirection direction;
    private Timer releaseTimer;
    private Timer expirationTimer;

    public CoralizerIntakeCommand(ShooterSubsystem shooterSubsystem, IntakeDirection direction) {
        this.shooterSubsystem = shooterSubsystem;
        this.direction = direction;
        addRequirements(shooterSubsystem);

        releaseTimer = new Timer();
        expirationTimer = new Timer();
    }

    @Override
    public void initialize() {
        super.initialize();
        shooterSubsystem.setShouldHold(false);
        switch (direction){
            case IN -> shooterSubsystem.setIntakeSpeed(-1);
            case OUT, ALGAE -> shooterSubsystem.setIntakeSpeed(0.9);
            case SHOOT -> shooterSubsystem.setIntakeSpeed(1);
            case L1INTAKE -> shooterSubsystem.setIntakeSpeed(0.6);
        }
        releaseTimer.reset();
        expirationTimer.reset();
        expirationTimer.start();
    }

    @Override
    public void execute() {
        if((direction == IntakeDirection.IN && shooterSubsystem.hasCoral()) || (direction != IntakeDirection.IN && !shooterSubsystem.hasCoral())) {releaseTimer.start();}
    }

    @Override
    public boolean isFinished() {
        if(releaseTimer.get() >= 0.25 && direction == IntakeDirection.IN){
            return shooterSubsystem.hasCoral();
        } else if (releaseTimer.get() >= 0.6 && direction == IntakeDirection.OUT) {
            return !shooterSubsystem.hasCoral();
            //return false;
        }
//        else if (direction == IntakeDirection.ALGAE) {
//            return shooterSubsystem.hasAlgae();
//        }
        return false;

//        if (expirationTimer.get() >= 7) {return true;}
//        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooterSubsystem.setIntakeSpeed(-0.03);
        expirationTimer.reset();
        releaseTimer.reset();
    }

    public static enum IntakeDirection{
        IN,
        OUT,
        SHOOT,
        ALGAE,
        L1INTAKE
    }

}
