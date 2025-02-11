package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.pathing.PathFindThenAccuratePathCommand;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AllignSourceCommandGroup extends SequentialCommandGroup {
    public AllignSourceCommandGroup(Pose2d sourcePos, PathConstraints constraints, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        addCommands(
                new PathFindThenAccuratePathCommand(sourcePos, constraints, s),
                new SourceCommandGroup(e, c)
        );

    }

}
