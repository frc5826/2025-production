package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.elevator.ElevatorRepositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.commands.coralizer.CoralizerIntakeCommand.IntakeDirection.*;

public class DropoffCommandGroup extends SequentialCommandGroup {

    public DropoffCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem){
        addCommands(
                new ElevatorRepositionCommand(elevatorSubsystem, -0.3)
                //new CoralizerIntakeCommand(coralizerSubsystem, OUT)
        );
    }

}
