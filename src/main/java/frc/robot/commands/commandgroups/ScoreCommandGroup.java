package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.reef.ReefCommand;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.drivercontrol.TeleopDriveCommand;
import frc.robot.commands.swerve.pathing.PathFindCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ReefTargeting;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.BluePositions.cRobotLength;

public class ScoreCommandGroup extends SequentialCommandGroup {

    public ScoreCommandGroup(ReefTargeting target, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        Pose2d endPose = MathHelper.offsetPoseReverse(target.getPose(), cRobotLength / 2);
        Pose2d offsetPose = MathHelper.offsetPoseReverse(target.getPose(), 0.35 + (cRobotLength / 2));

        PathConstraints alignConstraints = new PathConstraints(1.5, 1.5, Math.PI * 2, Math.PI * 2);
        PathConstraints fastConstraints = new PathConstraints(3, 4, Math.PI * 3, Math.PI * 4);

        addCommands(

                new PathFindCommand(offsetPose, fastConstraints, s)
                        .onlyWhile(() -> s.getLocalizationPose().getTranslation().getDistance(offsetPose.getTranslation()) > 1.5),
                Commands.deadline(
                        Commands.parallel(
                                new PathFindCommand(offsetPose, fastConstraints, s),
                                new ElevatorPositionCommand(e, 1.07, target.getLevel())
                        ),
                        new CoralizerWristCommand(c, target.getLevel().angle).onlyWhile(() -> e.getPos() > 0.5)
                ),
                Commands.parallel(
                        new PathToCommand(endPose, 0, alignConstraints, s),
                        new CoralizerWristCommand(c, target.getLevel().angle)
                ),
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT),
                Commands.parallel(
                        new HomeCommandGroup(e, c),
                        new TeleopDriveCommand(s)
                )

        );

    }

}
