package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SourceCommandGroup extends SequentialCommandGroup {

    public SourceCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

        addCommands(
                new ElevatorPositionCommand(elevatorSubsystem, 0),
                new CoralizerWristCommand(coralizerSubsystem, 68)
        );

    }

}
