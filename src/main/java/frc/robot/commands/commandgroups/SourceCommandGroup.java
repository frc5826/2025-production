package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SourceCommandGroup extends SequentialCommandGroup {

    public SourceCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

        addCommands(
                Commands.parallel(
                        new ElevatorPositionCommand(elevatorSubsystem, 0.24, ReefPosition.ReefLevel.NONE),
                        new CoralizerWristCommand(coralizerSubsystem, 25)
                ),
                new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.IN),
                new MovingHeightCommandGroup(elevatorSubsystem, coralizerSubsystem)
        );

    }

}
