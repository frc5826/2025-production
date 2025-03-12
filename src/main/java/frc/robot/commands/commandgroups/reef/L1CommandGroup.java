package frc.robot.commands.commandgroups.reef;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L1CommandGroup extends SequentialCommandGroup {

    public L1CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

         addCommands(
                 new ElevatorPositionCommand(elevatorSubsystem, Constants.Elevator.L1Height, ReefPosition.ReefLevel.L1),
                 new CoralizerWristCommand(coralizerSubsystem, -10)
        );

    }
}
