package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.pathing.PathFindThenAccuratePathCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
import frc.robot.subsystems.SwerveSubsystem;

public class Right extends SequentialCommandGroup {

    private SwerveSubsystem s;

    private Orientation o;

    private PathConstraints slowConstraints;
    private PathConstraints fastConstraints;

    public Right(SwerveSubsystem swerveSubsystem) {
        this.s = swerveSubsystem;

        slowConstraints = new PathConstraints(2, 2, Math.PI * 1.5, Math.PI * 2);
        fastConstraints = new PathConstraints(5, 4, Math.PI * 4, Math.PI * 5);

        o = FieldOrientation.getOrientation();


        //Drop off starting coral
        addCommands(
                Commands.parallel(
                        new PathToCommand(o.getReefE(), 0, slowConstraints, s)
                        //elevator up
                        //prep wrist
                )
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
