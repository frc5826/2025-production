package frc.robot.commands.commandgroups.algae;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.AlgaeIntakeCommand;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DealgifyL2CommandGroup extends SequentialCommandGroup {

    public DealgifyL2CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {

        addCommands(
                Commands.parallel(
                        new ElevatorPositionCommand(elevatorSubsystem, 0.55, ReefPosition.ReefLevel.NONE),
                        new CoralizerWristCommand(coralizerSubsystem, 10),
                        new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.IN)
                )
//                new AlgaeIntakeCommand(coralizerSubsystem),
//                new MoveTimeCommand(1, new ChassisSpeeds(-0.75, 0, 0), true, swerveSubsystem).asProxy(),
//                new ElevatorPositionCommand(elevatorSubsystem, 0.1, ReefPosition.ReefLevel.NONE)
        );

    }

}
