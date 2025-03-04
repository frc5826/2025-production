package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

public class PathToTwoPosesCommand extends Command {

    private SwerveSubsystem swerveSubsystem;

    private Pose2d mid;
    private Pose2d goal;
    private PathConstraints constraints;
    private double endVel;

    private Command pathCommand;

    public PathToTwoPosesCommand(Pose2d mid, Pose2d goal, double endVel, PathConstraints constraints, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        this.mid = mid;
        this.goal = goal;
        this.constraints = constraints;
        this.endVel = endVel;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        Pose2d start = swerveSubsystem.getLocalizationPose();
        start = new Pose2d(start.getTranslation(), MathHelper.getAngleAtoB(start, mid));
        mid = new Pose2d(mid.getTranslation(), MathHelper.getAngleAtoB(start , mid));
        Pose2d end = new Pose2d(goal.getTranslation(), MathHelper.getAngleAtoB(mid, goal));

        List<Waypoint> points = PathPlannerPath.waypointsFromPoses(start, mid, end);

        PathPlannerPath path = new PathPlannerPath(
                points,
                constraints,
                null,
                new GoalEndState(endVel, goal.getRotation()));

        pathCommand = AutoBuilder.followPath(path);

        pathCommand.initialize();

        Constants.cXbox.setRumble(GenericHID.RumbleType.kBothRumble, Constants.rumbleHigh);
    }

    @Override
    public void execute() {
        super.execute();
        pathCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        pathCommand.end(interrupted);

        Constants.cXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

}
