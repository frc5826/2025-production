package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorRepositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L3DropoffCommandGroup extends SequentialCommandGroup {

    public L3DropoffCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem){
        addCommands(
                new ElevatorRepositionCommand(elevatorSubsystem, -0.3)
        );
        addCommands(
                new CoralizerWristCommand(coralizerSubsystem, .5)
        );
    }

}
