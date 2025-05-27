package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.NuzzleUpCommand;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
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

        PathConstraints alignConstraints = new PathConstraints(1, 1, Math.PI * 1.5, Math.PI * 2);
        PathConstraints fastConstraints = new PathConstraints(3, 3, Math.PI * 2, Math.PI * 3);

        Pose2d dealgPose = getDealgTarget(target.getPose());


        addCommands(

                new PathFindCommand(target.getFindOffsetPose(), 0.25, fastConstraints, s)
                        .onlyIf(target.isFarEnoughToPath()), //TODO test
                Commands.deadline(
                        Commands.parallel(
                                new PathToCommand(target.getAlignmentOffsetPose(), 0, alignConstraints, s),
                                new ElevatorPositionCommand(e, () -> target.getLevel().get().height, target.getLevel().get())
                        ),
                        new CoralizerWristCommand(c, () -> target.getLevel().get().angle).onlyWhile(() -> e.getPos() > 0.5)
                ),
                Commands.parallel(
                        new PathToCommand(target.getAlignmentPose(), 0.25, alignConstraints, s),
                        new CoralizerWristCommand(c, () -> target.getLevel().get().angle)
                ),
                new NuzzleUpCommand(d, s, ca, new AprilTag(0), target.getLeft()),
                new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.OUT),
                Commands.deadline(
                        new MoveTimeCommand(0.35, new ChassisSpeeds(-1, 0, 0), true, s),
                        new HomeCommandGroup(e, c)
                ),
                new FastAlignReefCommand()

        );

    }

    private ReefTargeting getDealgTarget(Pose2d coralTarget, SwerveSubsystem swerveSubsystem) {
        ReefTargeting algTarget = new ReefTargeting(swerveSubsystem);

        if (coralTarget == FieldOrientation.getOrientation().getReefA() || coralTarget == FieldOrientation.getOrientation().getReefB()) {
            algTarget.updatePose(FieldOrientation.getOrientation().getReefSideAB());
            algTarget.updateLevel(ReefPosition.ReefLevel.ALGL2);
        } else if (coralTarget == FieldOrientation.getOrientation().getReefC() || coralTarget == FieldOrientation.getOrientation().getReefD()) {
            algTarget = FieldOrientation.getOrientation().getReefSideCD();
        } else if (coralTarget == FieldOrientation.getOrientation().getReefE() || coralTarget == FieldOrientation.getOrientation().getReefF()) {
            algTarget = FieldOrientation.getOrientation().getReefSideEF();
        } else if (coralTarget == FieldOrientation.getOrientation().getReefG() || coralTarget == FieldOrientation.getOrientation().getReefH()) {
            algTarget = FieldOrientation.getOrientation().getReefSideGH();
        } else if (coralTarget == FieldOrientation.getOrientation().getReefI() || coralTarget == FieldOrientation.getOrientation().getReefJ()) {
            algTarget = FieldOrientation.getOrientation().getReefSideIJ();
        } else if (coralTarget == FieldOrientation.getOrientation().getReefK() || coralTarget == FieldOrientation.getOrientation().getReefL()) {
            algTarget = FieldOrientation.getOrientation().getReefSideKL();
        } else {
            return null;
        }

        return algTarget;
    }

}
