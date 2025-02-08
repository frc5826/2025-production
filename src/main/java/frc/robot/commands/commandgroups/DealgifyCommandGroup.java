package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DealgifyCommandGroup extends SequentialCommandGroup {

    public DealgifyCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

        addCommands(
                new ElevatorPositionCommand(elevatorSubsystem, 0.61),
                new CoralizerWristCommand(coralizerSubsystem, 0)
        );

    }

}
