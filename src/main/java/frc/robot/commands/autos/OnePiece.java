package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.dropoff.DropoffCommandGroup;
import frc.robot.commands.commandgroups.reef.*;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.*;
import frc.robot.math.MathHelper;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.BluePositions.cRobotLength;

public class OnePiece extends SequentialCommandGroup {
    public OnePiece(ReefPosition reefGoal, Pose2d sourceGoal, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        PathConstraints lowAccConstraints = new PathConstraints(2, 0.75, Math.PI, Math.PI);
        PathConstraints slowConstraints = new PathConstraints(1, 1, Math.PI * 1.5, Math.PI * 2);
        PathConstraints fastConstraints = new PathConstraints(2.5, 3, Math.PI * 3, Math.PI * 4);

        Supplier<Command> levelCommand = () -> new L4CommandGroup(e, c);

        Pose2d offsetReefPose = MathHelper.offsetPoseReverse(reefGoal.getPosition(), 0.6 + (cRobotLength / 2));
        Pose2d goalReefPose = MathHelper.offsetPoseReverse(reefGoal.getPosition(), (cRobotLength / 2) - 0.02);

        //Path find
        addCommands(
                new PathFindCommand(offsetReefPose, fastConstraints, s)
        );

        //Choose level command
        switch (reefGoal.getLevel()){
            case L1 -> levelCommand = () -> new L1CommandGroup(e, c);
            case L2 -> levelCommand = () -> new L2CommandGroup(e, c);
            case L3 -> levelCommand = () -> new L3CommandGroup(e, c);
            case L4 -> levelCommand = () -> new L4CommandGroup(e, c);
        }

        addCommands( //Align while raising elevator
                Commands.deadline(
                        new PathToTwoPosesCommand(offsetReefPose, goalReefPose, 0, slowConstraints, s),
                        levelCommand.get()
                ),
                Commands.parallel(
                        new AccuratePathCommand(goalReefPose, 2, true, s),
                        levelCommand.get()
                )
        );

        addCommands( //Drop off and backup
                //new DropoffCommandGroup(e, c, s), //TODO make autos work like tele cycles
                new MoveTimeCommand(0.75, new ChassisSpeeds(-1.5, 0, 0), true, s)
        );

        //untested optimized version
//        addCommands(
//                new CoralizerWristCommand(c, -20).withTimeout(1),
//                new ElevatorRepositionCommand(e, -0.3, ElevatorSubsystem.LevelTarget.NONE),
//                Commands.parallel(
//                        new CoralizerIntakeCommand(c, OUT),
//                        new MoveTimeCommand(0.75, new ChassisSpeeds(-0.75, 0, 0), true, s)
//                ),
//                Commands.deadline(
//                        new HomeCommandGroup(e, c),
//                        new PathFindCommand(sourceGoal, lowAccConstraints, s)
//                )
//        );
    }
}
