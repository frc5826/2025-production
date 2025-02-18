package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoGroundPickupCommand extends SequentialCommandGroup {
    public AutoGroundPickupCommand(ElevatorSubsystem e, CoralizerSubsystem c) {

        addCommands(
                Commands.parallel(
                        new CoralizerWristCommand(c, -18),
                        new ElevatorPositionCommand(e, 0, ElevatorSubsystem.LevelTarget.NONE)
                ),
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.IN),
                new HomeCommandGroup(e, c)
        );

    }
}
