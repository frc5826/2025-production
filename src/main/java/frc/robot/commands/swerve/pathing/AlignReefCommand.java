package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.*;
import static frc.robot.Constants.BluePositions.cPipeApart;
import static frc.robot.Constants.BluePositions.cRobotLength;

public class AlignReefCommand extends SequentialCommandGroup {

    public AlignReefCommand(Pose2d reefPose, PathConstraints fastConstraints, SwerveSubsystem s) {

        PathConstraints slowConstraints = new PathConstraints(0.5, 0.75, Math.PI, Math.PI * 2);

        Pose2d offsetReefPose = MathHelper.offsetPoseReverse(reefPose, 0.6 + (cRobotLength / 2));

        addCommands(
                new PathFindCommand(offsetReefPose, fastConstraints, s),
                new PathToTwoPosesCommand(offsetReefPose, MathHelper.offsetPoseReverse(reefPose, (cRobotLength / 2) - 0.03), 0, slowConstraints, s),
                new AccuratePathCommand(MathHelper.offsetPoseReverse(reefPose, (cRobotLength / 2) - 0.03), 1.5, true, s)
        );

    }

}
