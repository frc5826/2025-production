package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.*;
import frc.robot.commands.commandgroups.reef.*;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.AccuratePathCommand;
import frc.robot.commands.swerve.pathing.PathFindCommand;
import frc.robot.commands.swerve.pathing.PathOffsetThenAccurateCommand;
import frc.robot.commands.swerve.pathing.PathToTwoPosesCommand;
import frc.robot.math.MathHelper;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.cL4offset;

public class OnePiece extends SequentialCommandGroup {
    public OnePiece(ReefPosition reefGoal, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        PathConstraints slowConstraints = new PathConstraints(1, 2, Math.PI * 1.5, Math.PI * 2);
        PathConstraints fastConstraints = new PathConstraints(2, 3, Math.PI * 3, Math.PI * 4);

        Command levelCommand = new L4CommandGroup(e, c, s);
        Command accuratePathCommand;
        boolean L4;

        //Path find
        //addCommands(new PathOffsetThenAccurateCommand(reefGoal.getPosition(), fastConstraints, 0.5, true, s));
        Pose2d offsetGoal = MathHelper.offsetPoseReverse(reefGoal.getPosition(), 0.5);
        addCommands(
                new PathFindCommand(offsetGoal, fastConstraints, s)
        );

        //Choose level command
        switch (reefGoal.getLevel()){
            case L1 -> levelCommand = new L1CommandGroup(e, c);
            case L2 -> levelCommand = new L2CommandGroup(e, c);
            case L3 -> levelCommand = new L3CommandGroup(e, c);
            case L4 -> levelCommand = new L4NoSwerveCommandGroup(e, c);
        }
        accuratePathCommand = levelCommand.equals(new L4NoSwerveCommandGroup(e, c)) ?
                new AccuratePathCommand(MathHelper.offsetPoseReverse(reefGoal.getPosition(), cL4offset), 2, s) :
                new AccuratePathCommand(reefGoal.getPosition(), 2, s);
        
        addCommands( //Align while raising elevator
                Commands.deadline(
                        new PathToTwoPosesCommand(offsetGoal, reefGoal.getPosition(), 0, slowConstraints, s),
                        levelCommand
                ),
                Commands.parallel(
                        accuratePathCommand,
                        levelCommand
                )
        );

        addCommands( //Drop off and backup
                new DropoffCommandGroup(e, c, s),
                new MoveTimeCommand(0.75, new ChassisSpeeds(-1.5, 0, 0), true, s)
        );
    }
}
