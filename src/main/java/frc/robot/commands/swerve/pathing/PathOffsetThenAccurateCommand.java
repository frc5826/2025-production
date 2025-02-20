package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PathOffsetThenAccurateCommand extends SequentialCommandGroup {

    public PathOffsetThenAccurateCommand(Pose2d goal, PathConstraints constraints, double offset, SwerveSubsystem s) {

        PathConstraints slowConstraints = new PathConstraints(0.75, 1, Math.PI, Math.PI * 2);

        addCommands(
                new PathFindCommand(MathHelper.offsetPoseReverse(goal, offset), constraints, s),
                new PathToCommand(goal, 0, slowConstraints, s)
        );

    }

}
