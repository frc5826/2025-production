package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.AlignSourceCommandGroup;
import frc.robot.commands.commandgroups.DropoffCommandGroup;
import frc.robot.commands.commandgroups.L4CommandGroup;
import frc.robot.commands.swerve.pathing.PathOffsetThenAccurateCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(Pose2d firstGoal, Pose2d secondGoal, Pose2d sourceGoal, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        PathConstraints slowConstraints = new PathConstraints(1, 2, Math.PI * 1.5, Math.PI * 2);
        PathConstraints fastConstraints = new PathConstraints(2, 3, Math.PI * 3, Math.PI * 4);

        //Path to reef and drop off
        addCommands(
                new PathOffsetThenAccurateCommand(firstGoal, fastConstraints, 0.75, true, s),
                new L4CommandGroup(e, c, s),
                new DropoffCommandGroup(e, c, s)
        );
        //TODO Fix This
        //Path to source and intake
        addCommands(
                //new AlignSourceCommandGroup(sourceGoal, fastConstraints, s, e, c)
        );

        //Path to reef and drop off
        addCommands(
                new PathOffsetThenAccurateCommand(secondGoal, fastConstraints, 0.75, true, s),
                new L4CommandGroup(e, c, s),
                new DropoffCommandGroup(e, c, s)
        );

    }
}
