package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.algae.HomeAlgaeCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.drivercontrol.TeleopDriveCommand;
import frc.robot.commands.swerve.pathing.FastAlignReefCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.*;

public class DealgCommandGroup extends SequentialCommandGroup {

    public DealgCommandGroup(ReefTargeting target, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c, ShooterSubsystem sh) {

        PathConstraints constraints = new PathConstraints(1, 1, Math.PI * 1.5, Math.PI * 2);

        ReefTargeting dealgTarget = getDealgTarget(target.getTarget().getPosition(), s);

        addCommands(

                Commands.parallel(
                        new PathToCommand(dealgTarget.getAlignmentOffsetPose().get(), 0.05, constraints, s),
                        new ElevatorPositionCommand(e, dealgTarget.getTarget().getLevel().height, target.getTarget().getLevel()),
                        new CoralizerWristCommand(c, dealgTarget.getTarget().getLevel().angle)
                ),
                Commands.parallel(
                        new FastAlignReefCommand(dealgTarget.getAlignmentPose(), 1, s),
                        new InstantCommand(() -> sh.setIntakeSpeed(0.9))
                ),
                Commands.parallel(
                        new MoveTimeCommand(0.6, new ChassisSpeeds(-1, 0, 0), true, s),
                        new InstantCommand(() -> sh.setIntakeSpeed(0.9))
                ),
                Commands.parallel(
                        new TeleopDriveCommand(s),
                        new HomeAlgaeCommandGroup(e, c, sh)
                ),
                new InstantCommand(() -> sh.setIntakeSpeed(0.9))

        );

    }

    private ReefTargeting getDealgTarget(Pose2d coralTarget, SwerveSubsystem swerveSubsystem) {
        ReefTargeting algTarget = new ReefTargeting(swerveSubsystem);

        if (coralTarget == FieldOrientation.getOrientation().getReefA() || coralTarget == FieldOrientation.getOrientation().getReefB()) {
            algTarget.updatePose(FieldOrientation.getOrientation().getReefSideAB());
            algTarget.updateLevel(ReefPosition.ReefLevel.ALGL2);
        } else if (coralTarget == FieldOrientation.getOrientation().getReefC() || coralTarget == FieldOrientation.getOrientation().getReefD()) {
            algTarget.updatePose(FieldOrientation.getOrientation().getReefSideCD());
            algTarget.updateLevel(ReefPosition.ReefLevel.ALGL3);
        } else if (coralTarget == FieldOrientation.getOrientation().getReefE() || coralTarget == FieldOrientation.getOrientation().getReefF()) {
            algTarget.updatePose(FieldOrientation.getOrientation().getReefSideEF());
            algTarget.updateLevel(ReefPosition.ReefLevel.ALGL2);
        } else if (coralTarget == FieldOrientation.getOrientation().getReefG() || coralTarget == FieldOrientation.getOrientation().getReefH()) {
            algTarget.updatePose(FieldOrientation.getOrientation().getReefSideGH());
            algTarget.updateLevel(ReefPosition.ReefLevel.ALGL3);
        } else if (coralTarget == FieldOrientation.getOrientation().getReefI() || coralTarget == FieldOrientation.getOrientation().getReefJ()) {
            algTarget.updatePose(FieldOrientation.getOrientation().getReefSideIJ());
            algTarget.updateLevel(ReefPosition.ReefLevel.ALGL2);
        } else if (coralTarget == FieldOrientation.getOrientation().getReefK() || coralTarget == FieldOrientation.getOrientation().getReefL()) {
            algTarget.updatePose(FieldOrientation.getOrientation().getReefSideKL());
            algTarget.updateLevel(ReefPosition.ReefLevel.ALGL3);
        } else {
            algTarget.updatePose(FieldOrientation.getOrientation().getReefSideAB());
            algTarget.updateLevel(ReefPosition.ReefLevel.ALGL2);
        }

        return algTarget;
    }

}
