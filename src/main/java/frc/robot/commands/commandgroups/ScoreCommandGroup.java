package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.NuzzleUpCommand;
import frc.robot.commands.commandgroups.reef.ReefCommand;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.drivercontrol.TeleopDriveCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.PathFindCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.math.MathHelper;
import frc.robot.positioning.AprilTag;
import frc.robot.subsystems.*;

import static frc.robot.Constants.BluePositions.cRobotLength;

public class ScoreCommandGroup extends SequentialCommandGroup {

    public ScoreCommandGroup(ReefTargeting target, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c, DistanceSubsystem d, CameraSubsystem ca) {

        PathConstraints alignConstraints = new PathConstraints(1.5, 1.5, Math.PI * 1.5, Math.PI * 2);
        PathConstraints fastConstraints = new PathConstraints(3, 3, Math.PI * 2, Math.PI * 3);

        addCommands(

                new PathFindCommand(target.getFindOffsetPose(), 0, fastConstraints, s)
                        .onlyIf(target.isFarEnoughToPath()), //TODO test
                Commands.deadline(
                        Commands.parallel(
                                new PathToCommand(target.getAlignmentOffsetPose(), 0, alignConstraints, s),
                                new ElevatorPositionCommand(e, () -> target.getLevel().get().height, target.getLevel().get())
                        ),
                        new CoralizerWristCommand(c, () -> target.getLevel().get().angle).onlyWhile(() -> e.getPos() > 0.5)
                ),
                Commands.parallel(
                        new PathToCommand(target.getAlignmentPose(), 0, alignConstraints, s),
                        new CoralizerWristCommand(c, () -> target.getLevel().get().angle)
                ),
                new NuzzleUpCommand(d, s, ca, new AprilTag(0), target.getLeft()),
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT),
                Commands.deadline(
                        new MoveTimeCommand(0.35, new ChassisSpeeds(-1, 0, 0), true, s),
                        new HomeCommandGroup(e, c)
                ),
                Commands.parallel(
                        new TeleopDriveCommand(s),
                        new HomeCommandGroup(e, c)
                )

        );

    }

}
