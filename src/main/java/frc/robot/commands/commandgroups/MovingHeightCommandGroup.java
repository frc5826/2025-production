package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class MovingHeightCommandGroup extends SequentialCommandGroup {

    public MovingHeightCommandGroup(ElevatorSubsystem e, CoralizerSubsystem c) {

        addCommands(
                new ElevatorPositionCommand(e, 0.55, ReefPosition.ReefLevel.NONE),
                new CoralizerWristCommand(c, 30)
        );

    }

}
