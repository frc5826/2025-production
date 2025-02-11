package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L1CommandGroup extends SequentialCommandGroup {

    public L1CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

         addCommands(
                 new InstantCommand(() -> elevatorSubsystem.setElevatorTarget(ElevatorSubsystem.ElevatorTarget.L1)),
                 new ElevatorPositionCommand(elevatorSubsystem, 0),
                 new CoralizerWristCommand(coralizerSubsystem, 45)
        );

    }
}
