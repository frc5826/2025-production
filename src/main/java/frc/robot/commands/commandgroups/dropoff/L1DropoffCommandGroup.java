package frc.robot.commands.commandgroups.dropoff;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.HomeCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.commands.coralizer.CoralizerIntakeCommand.IntakeDirection.OUT;

public class L1DropoffCommandGroup extends SequentialCommandGroup {

    public L1DropoffCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
//                new ElevatorPositionCommand(elevatorSubsystem, 0.5, ElevatorSubsystem.LevelTarget.NONE),
//                new CoralizerWristCommand(coralizerSubsystem, -20),
                new CoralizerIntakeCommand(shooterSubsystem, OUT),
                new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem)
        );
    }

}
