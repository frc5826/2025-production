package frc.robot.commands.commandgroups.algae;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.coralizer.HoldAlgaeCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HomeAlgaeCommandGroup extends SequentialCommandGroup {

    public HomeAlgaeCommandGroup(ElevatorSubsystem e, CoralizerSubsystem c, ShooterSubsystem sh) {

        addCommands(
                
                Commands.parallel(
                        Commands.sequence(
                                new ElevatorPositionCommand(e, 0, ReefPosition.ReefLevel.NONE),
                                new CoralizerWristCommand(c, 30)
                                ),
                        new HoldAlgaeCommand(sh)
                )

        );

    }

}
