package frc.robot.commands.commandgroups;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.algae.HomeAlgaeCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.swerve.drivercontrol.TeleopDriveCommand;
import frc.robot.commands.swerve.pathing.FastAlignReefCommand;
import frc.robot.commands.swerve.pathing.MoveTimeCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.*;

public class DealgCommandGroup extends SequentialCommandGroup {

    public DealgCommandGroup(ReefTargeting target, SwerveSubsystem s, ElevatorSubsystem e, CoralizerSubsystem c, ShooterSubsystem sh) {

        PathConstraints constraints = new PathConstraints(1, 1, Math.PI * 1.5, Math.PI * 2);

        addCommands(

                Commands.parallel(
                        new PathToCommand(target.getAlignmentOffsetPose().get(), 0.05, constraints, s),
                        new ElevatorPositionCommand(e, target.getTarget().getLevel().height, target.getTarget().getLevel()),
                        new CoralizerWristCommand(c, target.getTarget().getLevel().angle)
                ),
                Commands.parallel(
                        new FastAlignReefCommand(target.getAlignmentPose(), 1.5, s),
                        new InstantCommand(() -> sh.setIntakeSpeed(0.9))
                ),
                Commands.parallel(
                        new MoveTimeCommand(0.6, new ChassisSpeeds(-1, 0, 0), true, s),
                        new InstantCommand(() -> sh.setIntakeSpeed(0.9))
                ),
                Commands.parallel(
                        new TeleopDriveCommand(s),
                        new HomeAlgaeCommandGroup(e, c, sh)
                )

        );

    }

}
