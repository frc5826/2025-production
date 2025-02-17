package frc.robot.commands.commandgroups;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class L4CommandGroup extends SequentialCommandGroup {

    public L4CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem) {

         addCommands(
                 new CoralizerWristCommand(coralizerSubsystem, 75),
                 Commands.parallel(
                         new MoveTimeCommand(0.2, new ChassisSpeeds(-0.65, 0, 0), true, swerveSubsystem),
                         new InstantCommand(() -> elevatorSubsystem.setElevatorTarget(ElevatorSubsystem.ElevatorTarget.L4))
                 ),
                 new ElevatorPositionCommand(elevatorSubsystem, 1.71),
                 new CoralizerWristCommand(coralizerSubsystem, 0)
         );
    }
}
