package frc.robot.commands.commandgroups.reef;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L3CommandGroup extends SequentialCommandGroup {

    public L3CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

         addCommands(
                 new CoralizerWristCommand(coralizerSubsystem, 30),
                 new ElevatorPositionCommand(elevatorSubsystem, Constants.Elevator.L3Height, ElevatorSubsystem.LevelTarget.L3),
                 new CoralizerWristCommand(coralizerSubsystem, -66)
         );

    }
}
