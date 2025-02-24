package frc.robot.commands.commandgroups.reef;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.pathing.AccuratePathCommand;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class L4NoSwerveCommandGroup extends SequentialCommandGroup {

    public L4NoSwerveCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

         addCommands(
                 new CoralizerWristCommand(coralizerSubsystem, 75),
                 new ElevatorPositionCommand(elevatorSubsystem, 1.71, ElevatorSubsystem.LevelTarget.L4),
                 new CoralizerWristCommand(coralizerSubsystem, 0)
         );
    }
}
