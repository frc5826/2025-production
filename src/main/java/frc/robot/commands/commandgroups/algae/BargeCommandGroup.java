package frc.robot.commands.commandgroups.algae;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BargeCommandGroup extends SequentialCommandGroup {

    public BargeCommandGroup(ElevatorSubsystem e, CoralizerSubsystem c, ShooterSubsystem s) {
        addCommands(
                Commands.parallel(
                        Commands.sequence(
                                new CoralizerWristCommand(c, 65),
                                new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4)
                        ),
                        new CoralizerIntakeCommand(s, CoralizerIntakeCommand.IntakeDirection.ALGAE)
                )
                //new CoralizerIntakeCommand(s, CoralizerIntakeCommand.IntakeDirection.IN)

        );
    }

}
