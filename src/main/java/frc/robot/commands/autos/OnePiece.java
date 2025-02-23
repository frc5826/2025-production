package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.*;
import frc.robot.commands.swerve.pathing.PathOffsetThenAccurateCommand;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class OnePiece extends SequentialCommandGroup {
    public OnePiece(ReefPosition reefGoal, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c) {

        PathConstraints slowConstraints = new PathConstraints(1, 2, Math.PI * 1.5, Math.PI * 2);
        PathConstraints fastConstraints = new PathConstraints(2, 3, Math.PI * 3, Math.PI * 4);

        //Path and drop off
        addCommands(new PathOffsetThenAccurateCommand(reefGoal.getPosition(), fastConstraints, 0.75, true, s));
        switch (reefGoal.getLevel()){
            case L1 -> addCommands(new L1CommandGroup(e, c));
            case L2 -> addCommands(new L2CommandGroup(e, c));
            case L3 -> addCommands(new L3CommandGroup(e, c));
            case L4 -> addCommands(new L4CommandGroup(e, c, s));
        }
        addCommands(new DropoffCommandGroup(e, c, s));
    }
}
