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

public class SourceCommandGroup extends SequentialCommandGroup {

    public SourceCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, ShooterSubsystem shooterSubsystem) {

        addCommands(
                Commands.parallel(
                        new ElevatorPositionCommand(elevatorSubsystem, Constants.Elevator.intakeHeight, ReefPosition.ReefLevel.NONE),
                        new CoralizerWristCommand(coralizerSubsystem, Constants.Elevator.intakeAngle)
                ),
                new CoralizerIntakeCommand(shooterSubsystem, CoralizerIntakeCommand.IntakeDirection.IN),
                new MovingHeightCommandGroup(elevatorSubsystem, coralizerSubsystem)
        );

    }

}
