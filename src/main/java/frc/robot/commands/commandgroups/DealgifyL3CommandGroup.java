package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DealgifyL3CommandGroup extends SequentialCommandGroup {

    public DealgifyL3CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

        addCommands(
                new InstantCommand(() -> elevatorSubsystem.setLevelTarget(ElevatorSubsystem.LevelTarget.NONE)),
                new ElevatorPositionCommand(elevatorSubsystem, 1.01),
                new CoralizerWristCommand(coralizerSubsystem, 0),
                new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.IN)
        );

    }

}
