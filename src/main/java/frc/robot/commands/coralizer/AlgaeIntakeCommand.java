package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AlgaeIntakeCommand extends LoggedCommand {

    private ShooterSubsystem shooterSubsystem;



    public AlgaeIntakeCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);


    }



    @Override
    public void initialize() {
        shooterSubsystem.setShouldHold(true);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.hasCoral();
    }


}



