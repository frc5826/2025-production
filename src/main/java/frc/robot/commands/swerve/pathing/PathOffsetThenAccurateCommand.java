package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PathOffsetThenAccurateCommand extends SequentialCommandGroup {

    public PathOffsetThenAccurateCommand(Pose2d goal, PathConstraints constraints, double offset, boolean reef, SwerveSubsystem s) {

        Pose2d offsetGoal = MathHelper.offsetPoseReverse(goal, offset);

        if (!reef) {
            offsetGoal = MathHelper.offsetPose(goal, offset, new Rotation2d(-Math.PI / 2));
        }

        PathConstraints slowConstraints = new PathConstraints(0.4, 1, Math.PI, Math.PI * 2);

        addCommands(
                new PathFindCommand(offsetGoal, constraints, s),
                new PathToTwoPosesCommand(offsetGoal, goal, 0, slowConstraints, s),
                new AccuratePathCommand(goal, 2, true, s)
        );

    }

}
