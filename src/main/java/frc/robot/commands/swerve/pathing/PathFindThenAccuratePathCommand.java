package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PathFindThenAccuratePathCommand extends SequentialCommandGroup {

    private SwerveSubsystem s;
    private CameraSubsystem ca;

    private Pose2d goal;
    private boolean singleTagAlign;
    private PathConstraints constraints;

    public PathFindThenAccuratePathCommand(Pose2d goal, boolean singleTagAlign, PathConstraints constraints,
                                           SwerveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem) {
        this.s = swerveSubsystem;
        this.ca = cameraSubsystem;

        this.goal = goal;
        this.singleTagAlign = singleTagAlign;
        this.constraints = constraints;

        addCommands(
                new PathFindCommand(goal, constraints, s),
                new AccuratePathCommand(goal, s)
        );
    }

}
