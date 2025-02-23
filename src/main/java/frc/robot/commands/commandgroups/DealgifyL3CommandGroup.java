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

public class DealgifyL3CommandGroup extends SequentialCommandGroup {

    public DealgifyL3CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem) {

        addCommands(
                new ElevatorPositionCommand(elevatorSubsystem, 1.01, ElevatorSubsystem.LevelTarget.NONE),
                new CoralizerWristCommand(coralizerSubsystem, 0),
                new AlgaeIntakeCommand(coralizerSubsystem),
                new MoveTimeCommand(2, new ChassisSpeeds(-0.5, 0, 0), true, swerveSubsystem),
                new ElevatorPositionCommand(elevatorSubsystem, 0.1, ElevatorSubsystem.LevelTarget.NONE)

        );


    }

}
