package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L2CommandGroup extends SequentialCommandGroup {

    private ElevatorSubsystem elevatorSubsystem;
    private CoralizerSubsystem coralizerSubsystem;

    public L2CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralizerSubsystem = coralizerSubsystem;

        addCommands(
                new ElevatorPositionCommand(elevatorSubsystem, 1),
                new CoralizerWristCommand(coralizerSubsystem, 0)
        );

    }
}
