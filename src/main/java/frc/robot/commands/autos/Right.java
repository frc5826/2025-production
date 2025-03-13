package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.commandgroups.MovingHeightCommandGroup;
import frc.robot.commands.commandgroups.SourceCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.math.MathHelper;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Right extends SequentialCommandGroup {

    public Right(SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        PathConstraints alignConstraints = new PathConstraints(1.5, 1.5, Math.PI * 1.5, Math.PI * 2);
        PathConstraints fastConstraints = new PathConstraints(4, 4, Math.PI * 2, Math.PI * 3);

        Command start = Commands.none();
        Command eToSource = Commands.none();
        Command sourceToCD = Commands.none();
        Command CDToSource = Commands.none();

        try {
            start = AutoBuilder.followPath(PathPlannerPath.fromPathFile("Right start"));
            eToSource = AutoBuilder.followPath(PathPlannerPath.fromPathFile("Right E to source"));
            sourceToCD = AutoBuilder.followPath(PathPlannerPath.fromPathFile("Right source to CD"));
            CDToSource = AutoBuilder.followPath(PathPlannerPath.fromPathFile("Right CD to source"));
        } catch (Exception exception) {
            System.err.println(exception.getMessage());
        }



        //First coral
        addCommands(
                Commands.parallel( //Path while prepping elevator and wrist
                        start,
                        new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4),
                        new CoralizerWristCommand(c, Constants.Elevator.L4Angle)
                ),
                new PathToCommand(MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefJ(), Constants.BluePositions.cRobotLength / 2), 0, alignConstraints, s), //Align to reef
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT) //Drop
        );

        //Get next coral
        addCommands(
                Commands.parallel(
                        eToSource,
                        new SourceCommandGroup(e, c)
                )
        );

        //Drop second coral
        addCommands(
                Commands.parallel(
                        sourceToCD,
                        new MovingHeightCommandGroup(e, c)
                ),
                Commands.parallel(
                        new PathToCommand(MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefL(), Constants.BluePositions.cRobotLength / 2), 0, alignConstraints, s), //Align to reef
                        new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4),
                        new CoralizerWristCommand(c, Constants.Elevator.L4Angle)
                ),
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT)
        );

        //Get next coral
        addCommands(
                Commands.parallel(
                        CDToSource,
                        new SourceCommandGroup(e, c)
                )
        );

        //Drop third coral
        addCommands(
                Commands.parallel(
                        sourceToCD,
                        new MovingHeightCommandGroup(e, c)
                ),
                Commands.parallel(
                        new PathToCommand(MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefK(), Constants.BluePositions.cRobotLength / 2), 0, alignConstraints, s), //Align to reef
                        new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4),
                        new CoralizerWristCommand(c, Constants.Elevator.L4Angle)
                ),
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT)
        );

        //Get next coral
        addCommands(
                Commands.parallel(
                        CDToSource,
                        new SourceCommandGroup(e, c)
                )
        );
    }

}
