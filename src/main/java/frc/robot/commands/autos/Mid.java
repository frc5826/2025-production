package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.commandgroups.HomeCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.math.MathHelper;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.BluePositions.cRobotLength;

public class Mid extends SequentialCommandGroup {

    public Mid(SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        PathConstraints alignConstraints = new PathConstraints(1.5, 1.5, Math.PI * 1.5, Math.PI * 2);

        addCommands(

                Commands.parallel( //Path while prepping elevator and wrist
                        new PathToCommand(MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefG(), 0.35 + (cRobotLength / 2)), 0, alignConstraints, s),
                        new ElevatorPositionCommand(e, Constants.Elevator.L4Height, ReefPosition.ReefLevel.L4),
                        new CoralizerWristCommand(c, Constants.Elevator.L4Angle)
                ),
                new PathToCommand(MathHelper.offsetPoseReverse(FieldOrientation.getOrientation().getReefG(), Constants.BluePositions.cRobotLength / 2), 0, alignConstraints, s), //Align to reef
                new CoralizerIntakeCommand(c, CoralizerIntakeCommand.IntakeDirection.OUT), //Drop
                Commands.parallel(
                        new MoveTimeCommand(0.35, new ChassisSpeeds(-1, 0, 0), true, s),
                        new HomeCommandGroup(e, c)
                )

        );

    }

}
