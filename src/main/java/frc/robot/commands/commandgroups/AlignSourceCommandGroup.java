package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.pathing.*;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;
import static frc.robot.Constants.BluePositions.*;

public class AlignSourceCommandGroup extends SequentialCommandGroup {
    public AlignSourceCommandGroup(Supplier<Pose2d> sourcePos, PathConstraints constraints,
                                   SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        Pose2d offsetPose = MathHelper.offsetPose(sourcePos.get(), 0.6 + (cRobotLength / 2), new Rotation2d(-Math.PI / 2));
        Pose2d goalPose = MathHelper.offsetPose(sourcePos.get(), (cRobotLength / 2) - 0.1, new Rotation2d(-Math.PI / 2));

        PathConstraints slowConstraints = new PathConstraints(1, 2, Math.PI, Math.PI * 2);

        addCommands(
                //new PathOffsetWrapper(sourcePos, constraints, 0.4, false, s),
                new PathFindCommand(offsetPose, constraints, s),
                new PathToCommand(goalPose, 0, slowConstraints, s),
                new SourceCommandGroup(e, c)
        );

    }

}
