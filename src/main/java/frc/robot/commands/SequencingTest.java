package frc.robot.commands;

import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SequencingTest extends ConditionalSequentialCommandGroup{

    public SequencingTest(ElevatorSubsystem e, CoralizerSubsystem c) {
        addCommands(new ElevatorPositionCommand(e, 1.5, ElevatorSubsystem.LevelTarget.NONE));
        addCommands(new ConditionallyActivatedCommand(new CoralizerWristCommand(c, 0), () -> e.getPos() >= 0.5));
    }
}
