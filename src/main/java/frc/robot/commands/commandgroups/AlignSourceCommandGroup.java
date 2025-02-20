package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.pathing.PathFindThenAccuratePathCommand;
import frc.robot.commands.swerve.pathing.PathOffsetThenAccurateCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignSourceCommandGroup extends SequentialCommandGroup {
    public AlignSourceCommandGroup(Pose2d sourcePos, PathConstraints constraints,
                                   SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c, CameraSubsystem ca) {

        addCommands(
                new PathOffsetThenAccurateCommand(sourcePos, constraints, 0.75, s),
                new SourceCommandGroup(e, c)
        );

    }

}
