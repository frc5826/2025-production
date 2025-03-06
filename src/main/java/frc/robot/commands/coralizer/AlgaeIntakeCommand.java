package frc.robot.commands.coralizer;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.CoralizerSubsystem;

public class AlgaeIntakeCommand extends LoggedCommand {

    private CoralizerSubsystem coralizerSubsystem;



    public AlgaeIntakeCommand(CoralizerSubsystem coralizerSubsystem) {
        this.coralizerSubsystem = coralizerSubsystem;

        addRequirements(coralizerSubsystem);


    }



    @Override
    public void initialize() {
        coralizerSubsystem.setShouldHold(true);
    }

    @Override
    public boolean isFinished() {
        return coralizerSubsystem.hasCoral();
    }


}



