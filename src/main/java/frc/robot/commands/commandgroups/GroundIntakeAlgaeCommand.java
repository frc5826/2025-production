package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class GroundIntakeAlgaeCommand extends SequentialCommandGroup {

    public GroundIntakeAlgaeCommand(CoralizerSubsystem c, ElevatorSubsystem e) {

        addCommands(
                Commands.parallel(
                        new ElevatorPositionCommand(e, 0, ElevatorSubsystem.LevelTarget.NONE),
                        new CoralizerWristCommand(c, -15)
                ),
                new AlgaeIntakeCommand(c),
                new ElevatorPositionCommand(e, 0.1, ElevatorSubsystem.LevelTarget.NONE)
        );

    }

}
