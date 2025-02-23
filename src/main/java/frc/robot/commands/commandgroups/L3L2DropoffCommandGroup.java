package frc.robot.commands.commandgroups;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorRepositionCommand;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class L3L2DropoffCommandGroup extends SequentialCommandGroup {

    public L3L2DropoffCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem){
        addCommands(

                Commands.parallel(
                new ElevatorRepositionCommand(elevatorSubsystem, -0.3, ElevatorSubsystem.LevelTarget.NONE),
                new CoralizerWristCommand(coralizerSubsystem, 20)
                        ),
                Commands.parallel(
                    new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.OUT),
                    new MoveTimeCommand(0.75, new ChassisSpeeds(-0.5, 0, 0), true, swerveSubsystem)
                ),
                new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem)
        );
    }

}
