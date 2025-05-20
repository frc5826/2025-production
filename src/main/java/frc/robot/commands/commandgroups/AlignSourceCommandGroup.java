package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.pathing.*;
import frc.robot.math.MathHelper;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignSourceCommandGroup extends SequentialCommandGroup {
//    public AlignSourceCommandGroup(Supplier<Pose2d> sourcePos, PathConstraints constraints,
//                                   SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {
//
//        Pose2d offsetPose = MathHelper.offsetPose(sourcePos.get(), 0.6 + (cRobotWidth / 2), new Rotation2d(-Math.PI / 2));
//        Pose2d goalPose = MathHelper.offsetPose(sourcePos.get(), (cRobotWidth / 2) - 0.1, new Rotation2d(-Math.PI / 2));
//
//        PathConstraints slowConstraints = new PathConstraints(1.5, 2, Math.PI, Math.PI * 2);
//
//        addCommands(
//                new PathFindCommand(() -> offsetPose, constraints, s),
//                Commands.parallel(
//                        new PathToCommand(goalPose, 0, slowConstraints, s),
//                        new SourceCommandGroup(e, c)
//                )
//        );
//
//    }

    public AlignSourceCommandGroup(Pose2d goal, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c, ShooterSubsystem sh) {

        PathConstraints constraints = new PathConstraints(3, 3, Math.PI * 2, Math.PI * 3);
        PathConstraints slowConstraints = new PathConstraints(2, 2, Math.PI * 2, Math.PI * 3);

        addCommands(

                new PathFindCommand(MathHelper.offsetPoseReverse(goal, 1), 0.5, constraints, s),
                Commands.parallel(
                        new PathToCommand(MathHelper.offsetPoseReverse(goal, (Constants.BluePositions.cRobotLength / 2) - .05), 0, slowConstraints, s),
                        new ElevatorPositionCommand(e, Constants.Elevator.intakeHeight, ReefPosition.ReefLevel.NONE),
                        Commands.sequence(
                                new CoralizerWristCommand(c, Constants.Elevator.intakeAngle),
                                new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.IN)
                        )

                )

        );

    }

}
