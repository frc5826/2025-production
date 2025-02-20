package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.HomeCommandGroup;
import frc.robot.commands.commandgroups.L4DropoffCommandGroup;
import frc.robot.commands.commandgroups.L4CommandGroup;
import frc.robot.commands.commandgroups.SourceCommandGroup;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.PathFindThenAccuratePathCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Right extends SequentialCommandGroup {

    private Orientation o;

    private PathConstraints slowConstraints;
    private PathConstraints fastConstraints;

    public Right(SwerveSubsystem s, ElevatorSubsystem e,
                 CoralizerSubsystem c, CameraSubsystem ca) {

        slowConstraints = new PathConstraints(2, 2, Math.PI * 1.5, Math.PI * 2);
        fastConstraints = new PathConstraints(5, 4, Math.PI * 4, Math.PI * 5);

        o = FieldOrientation.getOrientation();

        //Path and prep elevator
        addCommands(
                Commands.parallel(
                        new PathToCommand(o.getReefE(), 0, slowConstraints, s),
                        new L4CommandGroup(e, c, s)
                )
        );

        //Drop off coral
        addCommands(
                new L4DropoffCommandGroup(e, c, s)
        );

        //Path to coral station
        addCommands(
                Commands.parallel(
                        new HomeCommandGroup(e, c),
                        new PathFindThenAccuratePathCommand(o.getCoralStationRC(), false, fastConstraints, s, ca)
                )
        );

        //Pickup coral
        addCommands(
                new SourceCommandGroup(e, c)
        );

        //Path to reef and prep elevator
        addCommands(
                Commands.parallel(
                        new PathToCommand(o.getReefC(), 0, slowConstraints, s),
                        new L4CommandGroup(e, c, s)
                )
        );

        //Drop off coral
        addCommands(
                new L4DropoffCommandGroup(e, c, s)
        );

    }
}
