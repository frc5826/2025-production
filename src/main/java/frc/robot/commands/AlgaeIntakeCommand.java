package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
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



