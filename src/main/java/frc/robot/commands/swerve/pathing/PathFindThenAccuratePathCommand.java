package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class PathFindThenAccuratePathCommand extends SequentialCommandGroup {

    private SwerveSubsystem s;
    private Pose2d goal;
    private PathConstraints constraints;

    public PathFindThenAccuratePathCommand(Pose2d goal, PathConstraints constraints, SwerveSubsystem swerveSubsystem) {
        this.s = swerveSubsystem;
        this.goal = goal;
        this.constraints = constraints;

        addCommands(
                new PathFindCommand(goal, constraints, s),
                new AccuratePathCommand(goal, s)
        );
    }

}
