package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.pathing.PathFindThenAccuratePathCommand;
import frc.robot.commands.swerve.pathing.PathOffsetThenAccurateCommand;
import frc.robot.commands.swerve.pathing.PathOffsetWrapper;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class AlignSourceCommandGroup extends SequentialCommandGroup {
    public AlignSourceCommandGroup(Supplier<Pose2d> sourcePos, PathConstraints constraints,
                                   SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        addCommands(
                new PathOffsetWrapper(sourcePos, constraints, 0.4, false, s),
                new SourceCommandGroup(e, c)
        );

    }

}
