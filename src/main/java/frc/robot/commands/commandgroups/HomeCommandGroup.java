package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class HomeCommandGroup extends SequentialCommandGroup {

    public HomeCommandGroup(ElevatorSubsystem e, CoralizerSubsystem c){
        addCommands(
                new CoralizerWristCommand(c, 50),
                new ElevatorPositionCommand(e, 0, ElevatorSubsystem.LevelTarget.NONE)
        );
    }

}
