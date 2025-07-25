package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.NuzzleUpCommand;
import frc.robot.commands.commandgroups.MovingHeightCommandGroup;
import frc.robot.commands.commandgroups.SourceCommandGroup;
import frc.robot.commands.commandgroups.reef.L4CommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.pathing.FastAlignReefCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.math.MathHelper;
import frc.robot.positioning.AprilTag;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.*;

public class Left extends SequentialCommandGroup {

    private PathPlannerPath start;
    private PathPlannerPath jToSource;
    private PathPlannerPath sourceToKL;
    private PathPlannerPath KLToSource;

    public Left(SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c, DistanceSubsystem d, CameraSubsystem ca, ShooterSubsystem sh) {

        PathConstraints alignConstraints = Constants.Swerve.cAlignConstraints;
        PathConstraints fastConstraints = new PathConstraints(4, 4, Math.PI * 2, Math.PI * 3);

        try {
            start = PathPlannerPath.fromPathFile("Left start");
            jToSource = PathPlannerPath.fromPathFile("Left J to source");
            sourceToKL = PathPlannerPath.fromPathFile("Left source to KL");
            KLToSource = PathPlannerPath.fromPathFile("Left KL to source");
        } catch (Exception exception) {
            System.err.println(exception.getMessage());
        }

        //First coral
        addCommands(
                Commands.parallel( //Path while prepping elevator and wrist
                        buildPath(start),
                        new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4),
                        new CoralizerWristCommand(c, Constants.Elevator.L4Angle)
                ),
                new PathToCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefJ(), (Constants.BluePositions.cRobotLength / 2) - 0.03), 0.25, alignConstraints, s), //Align to reef,
                //new FastAlignReefCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefJ(), Constants.BluePositions.cRobotLength / 2), 1, s),
                new NuzzleUpCommand(d, s, ca, new AprilTag(0), () -> false), //Align to reef
                new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.OUT) //Drop
        );

        //Get next coral
        addCommands(
                Commands.parallel(
                        buildPath(jToSource),
                        new ElevatorPositionCommand(e, Constants.Elevator.intakeHeight, ReefPosition.ReefLevel.NONE),
                        new CoralizerWristCommand(c, Constants.Elevator.intakeAngle)
                ),
                Commands.deadline(
                        new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.IN),
                        new MoveTimeCommand(5, new ChassisSpeeds(0.3, 0, 0), true, s)
                )
        );

        //Drop second coral
        addCommands(
                Commands.parallel(
                        buildPath(sourceToKL),
                        new MovingHeightCommandGroup(e, c)
                ),
                Commands.deadline(
                        Commands.parallel(
                                new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4),
                                new CoralizerWristCommand(c, Constants.Elevator.L4Angle)
                        ),
                        new PathToCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefL(), (Constants.BluePositions.cRobotLength / 2) + 0.35), 0.25, alignConstraints, s)
                ),
                new PathToCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefL(), (Constants.BluePositions.cRobotLength / 2) - 0.03), 0.25, alignConstraints, s),
                //new FastAlignReefCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefL(), Constants.BluePositions.cRobotLength / 2), 1, s),
                new NuzzleUpCommand(d, s, ca, new AprilTag(0), () -> false), //Align to reef
                new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.OUT)
        );

        //Get next coral
        addCommands(
                Commands.parallel(
                        buildPath(KLToSource),
                        new ElevatorPositionCommand(e, Constants.Elevator.intakeHeight, ReefPosition.ReefLevel.NONE),
                        Commands.sequence(
                                new CoralizerWristCommand(c, Constants.Elevator.intakeAngle),
                                new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.IN)
                        )
                )
        );

        //Drop third coral
        addCommands(
                Commands.parallel(
                        buildPath(sourceToKL),
                        new MovingHeightCommandGroup(e, c)
                ),
                Commands.parallel(
                        new PathToCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefK(), (Constants.BluePositions.cRobotLength / 2) + 0.35), 0.1, alignConstraints, s),
                        new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4),
                        new CoralizerWristCommand(c, Constants.Elevator.L4Angle)
                ),
                new PathToCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefK(), (Constants.BluePositions.cRobotLength / 2) - 0.03), 0.25, alignConstraints, s),
                new NuzzleUpCommand(d, s, ca, new AprilTag(0), () -> true), //Align to reef
                new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.OUT)
        );

        //Get next coral
        addCommands(
                Commands.parallel(
                        buildPath(KLToSource),
                        new ElevatorPositionCommand(e, Constants.Elevator.intakeHeight, ReefPosition.ReefLevel.NONE),
                        Commands.sequence(
                                new CoralizerWristCommand(c, Constants.Elevator.intakeAngle),
                                new CoralizerIntakeCommand(sh, CoralizerIntakeCommand.IntakeDirection.IN)
                        )
                )
        );
    }

    private Command buildPath(PathPlannerPath path) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return AutoBuilder.followPath(path);
        } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return AutoBuilder.followPath(path.flipPath());
        } else {
            System.err.println("Auto paths set to none. No alliance found.");
            return Commands.none();
        }
    }

}
