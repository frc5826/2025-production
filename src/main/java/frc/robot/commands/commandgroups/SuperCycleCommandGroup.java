package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.NuzzleUpCommand;
import frc.robot.commands.commandgroups.algae.HomeAlgaeCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.coralizer.HoldAlgaeCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.drivercontrol.TeleopDriveCommand;
import frc.robot.commands.swerve.pathing.FastAlignReefCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.PathFindCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.positioning.AprilTag;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.*;

public class SuperCycleCommandGroup extends SequentialCommandGroup {

    public SuperCycleCommandGroup(ReefTargeting target, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c, ShooterSubsystem sh, CameraSubsystem ca, DistanceSubsystem d) {

        PathConstraints alignConstraints = Constants.Swerve.cAlignConstraints;
        PathConstraints fastConstraints = new PathConstraints(3, 3, Math.PI * 2, Math.PI * 3);

        addCommands(
//                new PathFindCommand(target.getFindOffsetPose(), 0.25, fastConstraints, s)
//                        .onlyIf(target.isFarEnoughToPath()),
                Commands.deadline(
                        new ElevatorPositionCommand(e, () -> target.getLevel().get().height, target.getLevel().get()),
                        new PathToCommand(target.getAlignmentOffsetPose(), 0.25, alignConstraints, s),
                        new CoralizerWristCommand(c, () -> target.getLevel().get().angle).onlyWhile(() -> e.getPos() > 0.5)
                ),
                Commands.parallel(
                        //new PathToCommand(target.getAlignmentPose(), 0.25, alignConstraints, s),
                        new FastAlignReefCommand(() -> target.getAlignmentPose().get(), 1, s), //TODO is this finishing or timing out?
                        new CoralizerWristCommand(c, () -> target.getLevel().get().angle)
                ),
                new NuzzleUpCommand(d, s, ca, new AprilTag(0), target.getLeft()),
                new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.OUT),
                new InstantCommand(target::getAlgaeTarget),
                Commands.deadline(
                        new MoveTimeCommand(0.35, new ChassisSpeeds(-1, 0, 0), true, s),
                        new ElevatorPositionCommand(e, () -> target.getTarget().getLevel().height, target.getTarget().getLevel())
                ),
                Commands.parallel(
                        new FastAlignReefCommand(() -> target.getAlignmentOffsetPose().get(), 1.5, s),
                        new ElevatorPositionCommand(e, () -> target.getTarget().getLevel().height, target.getTarget().getLevel()),
                        new CoralizerWristCommand(c, () -> target.getTarget().getLevel().angle)
                ),
                Commands.deadline(
                        new FastAlignReefCommand(() -> target.getAlignmentPose().get(), 1.5, s),
                        new HoldAlgaeCommand(sh)
                        //new InstantCommand(() -> sh.setIntakeSpeed(0.9))
                ),
                Commands.deadline(
                        new MoveTimeCommand(0.6, new ChassisSpeeds(-1, 0, 0), true, s),
                        new HoldAlgaeCommand(sh)
                        //new InstantCommand(() -> sh.setIntakeSpeed(0.9))
                ),
                Commands.parallel(
                        new TeleopDriveCommand(s),
                        new HomeAlgaeCommandGroup(e, c, sh)
                )

        );

    }

}
