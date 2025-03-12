package frc.robot.commands.commandgroups.dropoff;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.HomeCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class L3L2DropoffCommandGroup extends SequentialCommandGroup {

    public L3L2DropoffCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem){
        addCommands(
//                Commands.parallel(
//                        new ElevatorRepositionCommand(elevatorSubsystem, -0.3, ElevatorSubsystem.LevelTarget.NONE),
//                        new CoralizerWristCommand(coralizerSubsystem, 20)
//                ),
//                Commands.parallel(
//                    new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.OUT),
//                    new MoveTimeCommand(0.75, new ChassisSpeeds(-0.75, 0, 0), true, swerveSubsystem).asProxy()
//                ),
                new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.OUT),
                new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem)
        );
    }

}
