package frc.robot.commands.commandgroups.reef;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L4CommandGroup extends SequentialCommandGroup {

    public L4CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

         addCommands(
                 new CoralizerWristCommand(coralizerSubsystem, 30),
                 new ElevatorPositionCommand(elevatorSubsystem, Constants.Elevator.L4Height, ElevatorSubsystem.LevelTarget.L4),
                 new CoralizerWristCommand(coralizerSubsystem, -70)
         );

    }
}
