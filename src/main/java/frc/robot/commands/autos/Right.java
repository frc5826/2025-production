package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.NuzzleUpCommand;
import frc.robot.commands.commandgroups.MovingHeightCommandGroup;
import frc.robot.commands.commandgroups.SourceCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.math.MathHelper;
import frc.robot.positioning.AprilTag;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.*;

import java.nio.file.Path;

public class Right extends SequentialCommandGroup {

    private PathPlannerPath start;
    private PathPlannerPath eToSource;
    private PathPlannerPath sourceToCD;
    private PathPlannerPath CDToSource;

    public Right(SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c, DistanceSubsystem d, CameraSubsystem ca) {

        PathConstraints alignConstraints = new PathConstraints(1, 1, Math.PI * 1.5, Math.PI * 2);
        PathConstraints fastConstraints = new PathConstraints(4, 4, Math.PI * 2, Math.PI * 3);

        try {
            start = PathPlannerPath.fromPathFile("Right start");
            eToSource = PathPlannerPath.fromPathFile("Right E to source");
            sourceToCD = PathPlannerPath.fromPathFile("Right source to CD");
            CDToSource = PathPlannerPath.fromPathFile("Right CD to source");
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
                new PathToCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefE(), (Constants.BluePositions.cRobotLength / 2) - 0.03), 0.25, alignConstraints, s), //Align to reef
                new NuzzleUpCommand(d, s, ca, new AprilTag(0), () -> true), //Align to reef
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT) //Drop
        );

        //Get next coral
        addCommands(
                Commands.parallel(
                        buildPath(eToSource),
                        new ElevatorPositionCommand(e, Constants.Elevator.intakeHeight, ReefPosition.ReefLevel.NONE),
                        new CoralizerWristCommand(c, Constants.Elevator.intakeAngle)
                ),
                Commands.deadline(
                        new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.IN),
                        new MoveTimeCommand(5, new ChassisSpeeds(0.3, 0, 0), true, s)
                )
        );

        //Drop second coral
        addCommands(
                Commands.parallel(
                        buildPath(sourceToCD),
                        new MovingHeightCommandGroup(e, c)
                ),
                Commands.parallel(
                        new PathToCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefC(), (Constants.BluePositions.cRobotLength / 2) + 0.35), 0.1, alignConstraints, s),
                        new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4),
                        new CoralizerWristCommand(c, Constants.Elevator.L4Angle)
                ),
                new PathToCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefC(), (Constants.BluePositions.cRobotLength / 2) - 0.03), 0.25, alignConstraints, s),
                new NuzzleUpCommand(d, s, ca, new AprilTag(0), () -> true), //Align to reef
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT)
        );

        //Get next coral
        addCommands(
                Commands.parallel(
                        buildPath(CDToSource),
                        new ElevatorPositionCommand(e, Constants.Elevator.intakeHeight, ReefPosition.ReefLevel.NONE),
                        Commands.sequence(
                                new CoralizerWristCommand(c, Constants.Elevator.intakeAngle),
                                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.IN)
                        )
                )
        );

        //Drop third coral
        addCommands(
                Commands.parallel(
                        buildPath(sourceToCD),
                        new MovingHeightCommandGroup(e, c)
                ),
                Commands.parallel(
                        new PathToCommand(() -> MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefD(), Constants.BluePositions.cRobotLength / 2), 0, alignConstraints, s), //Align to reef
                        new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4),
                        new CoralizerWristCommand(c, Constants.Elevator.L4Angle)
                ),
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT)
        );

        //Get next coral
        addCommands(
                Commands.parallel(
                        buildPath(CDToSource),
                        new ElevatorPositionCommand(e, Constants.Elevator.intakeHeight, ReefPosition.ReefLevel.NONE),
                        Commands.sequence(
                                new CoralizerWristCommand(c, Constants.Elevator.intakeAngle),
                                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.IN)
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
