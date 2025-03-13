package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class PathFindCommand extends LoggedCommand {

    private SwerveSubsystem s;
    private Supplier<Pose2d> goal;
    private PathConstraints constraints;
    private Command command;

    public PathFindCommand(Supplier<Pose2d> goal, PathConstraints constraints, SwerveSubsystem swerveSubsystem) {
        this.s = swerveSubsystem;
        this.goal = goal;
        this.constraints = constraints;

        addRequirements(s);
    }

    public PathFindCommand(Pose2d goal, PathConstraints constraints, SwerveSubsystem swerveSubsystem) {
        this(() -> goal, constraints, swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        command = AutoBuilder.pathfindToPose(goal.get(), constraints);
        command.initialize();

        Constants.cXbox.setRumble(GenericHID.RumbleType.kBothRumble, Constants.rumbleHigh);
    }

    @Override
    public void execute() {
        super.execute();
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        command.end(interrupted);

        Constants.cXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }
}
