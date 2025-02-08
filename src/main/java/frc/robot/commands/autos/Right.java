package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.L4DropoffCommandGroup;
import frc.robot.commands.commandgroups.L4CommandGroup;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.PathFindThenAccuratePathCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Right extends SequentialCommandGroup {

    private SwerveSubsystem s;
    private ElevatorSubsystem e;
    private CoralizerSubsystem c;

    private Orientation o;

    private PathConstraints slowConstraints;
    private PathConstraints fastConstraints;

    public Right(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem,
                 CoralizerSubsystem coralizerSubsystem) {
        this.s = swerveSubsystem;

        slowConstraints = new PathConstraints(2, 2, Math.PI * 1.5, Math.PI * 2);
        fastConstraints = new PathConstraints(5, 4, Math.PI * 4, Math.PI * 5);

        o = FieldOrientation.getOrientation();


        //Path and prep elevator
        addCommands(
                Commands.parallel(
                        new PathToCommand(o.getReefE(), 0, fastConstraints, s),
                        new L4CommandGroup(e, c)
                )
        );

        //Drop off coral
        addCommands(
                new L4DropoffCommandGroup(e, c),
                new MoveTimeCommand(0.5, new ChassisSpeeds(-1, 0, 0), true, s)
        );

        //Path to coral station
        addCommands(
                new PathFindThenAccuratePathCommand(o.getCoralStationRC(), fastConstraints, s)
        );

        //Pickup coral
        addCommands(

        );

        //Path to reef
        addCommands(
                Commands.parallel(
                        new PathToCommand(o.getReefC(), 0, slowConstraints, s)
                        //raise elevator
                        //prep wrist
                )
        );

        //Drop off coral
        addCommands(

        );

    }
}
