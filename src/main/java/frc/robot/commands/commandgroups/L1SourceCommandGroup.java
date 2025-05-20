package frc.robot.commands.commandgroups;

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

public class L1SourceCommandGroup extends SequentialCommandGroup {

    public L1SourceCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, ShooterSubsystem shooterSubsystem) {

        addCommands(
                Commands.parallel(
                        new ElevatorPositionCommand(elevatorSubsystem, Constants.Elevator.L1IntakeHeight, ReefPosition.ReefLevel.NONE),
                        new CoralizerWristCommand(coralizerSubsystem, Constants.Elevator.L1IntakeAngle)
                ),
                new CoralizerIntakeCommand(shooterSubsystem, CoralizerIntakeCommand.IntakeDirection.L1INTAKE),
                new MovingHeightCommandGroup(elevatorSubsystem, coralizerSubsystem)
        );

    }

}
