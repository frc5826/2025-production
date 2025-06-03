package frc.robot.commands.commandgroups.algae;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ProcessorCommandGroup extends SequentialCommandGroup {

    public ProcessorCommandGroup(SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c, ShooterSubsystem sh) {

        addCommands(

                Commands.parallel(
                        new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.ALGAE),
                        new ElevatorPositionCommand(e, 0.1, ReefPosition.ReefLevel.NONE),
                        new CoralizerWristCommand(c, -10)
                )

        );

    }

}
