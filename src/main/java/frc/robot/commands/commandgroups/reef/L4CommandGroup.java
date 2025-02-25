package frc.robot.commands.commandgroups.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.AccuratePathCommand;
import frc.robot.commands.swerve.pathing.PathOffsetThenAccurateCommand;
import frc.robot.math.MathHelper;
import frc.robot.positioning.FieldOrientation;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.cL4offset;

public class L4CommandGroup extends SequentialCommandGroup {

    public L4CommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem) {

         addCommands(
                 new CoralizerWristCommand(coralizerSubsystem, 75),
                 //new AccuratePathCommand(() -> MathHelper.offsetPoseReverse(swerveSubsystem.getLocalizationPose(), cL4offset), 1, false, swerveSubsystem),
                 Commands.parallel(
                         //new MoveTimeCommand(0.3, new ChassisSpeeds(-0.4, 0, 0), true, swerveSubsystem)
                         new ElevatorPositionCommand(elevatorSubsystem, 1.71, ElevatorSubsystem.LevelTarget.L4)

                 ),
                 new CoralizerWristCommand(coralizerSubsystem, 0)
         );

    }
}
