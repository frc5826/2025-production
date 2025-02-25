package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreBargeCommandGroup extends SequentialCommandGroup {

    public ScoreBargeCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem){

        new ElevatorPositionCommand(elevatorSubsystem, 1.71, ElevatorSubsystem.LevelTarget.NONE);
        new CoralizerWristCommand(coralizerSubsystem, 45);
        new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.SHOOT);


    }

}
