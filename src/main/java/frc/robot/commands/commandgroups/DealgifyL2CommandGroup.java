package frc.robot.commands.commandgroups;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DealgifyL2CommandGroup extends SequentialCommandGroup {

    public DealgifyL2CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem) {

        addCommands(
                new ElevatorPositionCommand(elevatorSubsystem, 0.61, ElevatorSubsystem.LevelTarget.NONE),
                new CoralizerWristCommand(coralizerSubsystem, -10),
                new AlgaeIntakeCommand(coralizerSubsystem),
                new MoveTimeCommand(1, new ChassisSpeeds(-0.5, 0, 0), true, swerveSubsystem).asProxy(),
                new ElevatorPositionCommand(elevatorSubsystem, 0.1, ElevatorSubsystem.LevelTarget.NONE)
        );

    }

}
