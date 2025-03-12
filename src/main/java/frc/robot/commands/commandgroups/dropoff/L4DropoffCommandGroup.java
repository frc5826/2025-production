package frc.robot.commands.commandgroups.dropoff;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.HomeCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.commands.swerve.drivercontrol.TeleopDriveCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.commands.coralizer.CoralizerIntakeCommand.IntakeDirection.OUT;

public class L4DropoffCommandGroup extends SequentialCommandGroup {

    public L4DropoffCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem){
        addCommands(
//                Commands.deadline(
//                        new CoralizerWristCommand(coralizerSubsystem, -20).withTimeout(0.75),
//                        new TeleopDriveCommand(swerveSubsystem)
//                ),
//                Commands.deadline(
//                        new ElevatorRepositionCommand(elevatorSubsystem, -0.3, ElevatorSubsystem.LevelTarget.NONE),
//                        new TeleopDriveCommand(swerveSubsystem)
//                ),
//                Commands.parallel(
//                        new CoralizerIntakeCommand(coralizerSubsystem, OUT),
//                        new MoveTimeCommand(0.75, new ChassisSpeeds(-0.75, 0, 0), true, swerveSubsystem)
//                        //new AccuratePathCommand(() -> MathHelper.offsetPoseReverse(swerveSubsystem.getLocalizationPose(), 0.3), 1, false, swerveSubsystem)
//                ),
//                Commands.deadline(
//                        new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem),
//                        new TeleopDriveCommand(swerveSubsystem)
//                )

                new CoralizerIntakeCommand(coralizerSubsystem, OUT),
                new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem)
        );
    }

}
