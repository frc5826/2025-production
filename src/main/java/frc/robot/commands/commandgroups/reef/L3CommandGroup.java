package frc.robot.commands.commandgroups.reef;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L3CommandGroup extends SequentialCommandGroup {

    public L3CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

         addCommands(
                 new CoralizerWristCommand(coralizerSubsystem, 75),
                 new ElevatorPositionCommand(elevatorSubsystem, 0.82, ElevatorSubsystem.LevelTarget.L3),
                 new CoralizerWristCommand(coralizerSubsystem, 35)
         );

    }
}
