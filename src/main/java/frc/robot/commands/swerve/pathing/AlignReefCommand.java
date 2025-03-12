package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.HomeCommandGroup;
import frc.robot.commands.commandgroups.dropoff.DropoffCommandGroup;
import frc.robot.commands.commandgroups.reef.L3CommandGroup;
import frc.robot.commands.commandgroups.reef.L4CommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ReefTargeting;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.*;
import static frc.robot.Constants.BluePositions.cPipeApart;
import static frc.robot.Constants.BluePositions.cRobotLength;

public class AlignReefCommand extends SequentialCommandGroup {

//    public AlignReefCommand(Pose2d reefPose, PathConstraints fastConstraints, SwerveSubsystem s) {
//
//        PathConstraints slowConstraints = new PathConstraints(1.25, 1, Math.PI, Math.PI * 2);
//
//        Pose2d offsetReefPose = MathHelper.offsetPoseReverse(reefPose, 0.6 + (cRobotLength / 2));
//
//        addCommands(
//                new PathFindCommand(offsetReefPose, fastConstraints, s),
//                new PathToTwoPosesCommand(offsetReefPose, MathHelper.offsetPoseReverse(reefPose, (cRobotLength / 2)), 0, slowConstraints, s),
//                new AccuratePathCommand(MathHelper.offsetPoseReverse(reefPose, (cRobotLength / 2) - 0.03), 1.5, true, s)
//        );
//
//    }

    //testing sequence
//    public AlignReefCommand(Pose2d reefPose, Pose2d sidePose,
//                            SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {
//
//        Pose2d endPose = MathHelper.offsetPoseReverse(reefPose, cRobotLength / 2);
//        Pose2d offsetPose = MathHelper.offsetPoseReverse(reefPose, 0.35 + (cRobotLength / 2));
//        Pose2d farOffsetPose = MathHelper.offsetPoseReverse(reefPose, 1.5 + (cRobotLength / 2));
//
//        PathConstraints findConstraints = new PathConstraints(3, 4, Math.PI * 3, Math.PI * 3);
//        PathConstraints alignConstraints = new PathConstraints(1.5, 1.5, Math.PI * 2, Math.PI * 2);
//
//        addCommands(
//                Commands.deadline(
//                        Commands.parallel(
//                                new PathToCommand(offsetPose, 0, alignConstraints, s),
//                                new ElevatorPositionCommand(e, 1.07, ElevatorSubsystem.LevelTarget.L3)
//                        ),
//                        new CoralizerWristCommand(c, -66).onlyWhile(() -> e.getPos() > 0.5)
//                ),
//                Commands.parallel(
//                        new CoralizerWristCommand(c, -66),
//                        new PathToCommand(endPose, 0, alignConstraints, s)
//                ),
//                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT),
//                Commands.parallel(
//                        new HomeCommandGroup(e, c),
//                        new PathToCommand(farOffsetPose, 0, alignConstraints, s)
//                )
//
//        );
//
//    }

    //TODO add pathfinding
    public AlignReefCommand(ReefTargeting reefTargeting, SwerveSubsystem swerveSubsystem) {

        Pose2d endPose = MathHelper.offsetPoseReverse(reefTargeting.getPose(), cRobotLength / 2);
        Pose2d offsetPose = MathHelper.offsetPoseReverse(reefTargeting.getPose(), 0.35 + (cRobotLength / 2));

        PathConstraints alignConstraints = new PathConstraints(1.5, 1.5, Math.PI * 2, Math.PI * 2);

        addCommands(
                new PathToCommand(offsetPose, 0, alignConstraints, swerveSubsystem),
                new PathToCommand(endPose, 0, alignConstraints, swerveSubsystem)
        );

    }

}
