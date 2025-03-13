package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.MathHelper;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class PathToCommand extends LoggedCommand {

    private SwerveSubsystem swerveSubsystem;

    private Supplier<Pose2d> goal;
    private PathConstraints constraints;
    private double endVel;

    private Command pathCommand;

    public PathToCommand(Supplier<Pose2d> pose, double endVel, PathConstraints constraints, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        goal = pose;
        this.constraints = constraints;
        this.endVel = endVel;

        addRequirements(swerveSubsystem);
    }

    public PathToCommand(Pose2d pose, double endVel, PathConstraints constraints, SwerveSubsystem swerveSubsystem) {
        this(() -> pose, endVel, constraints, swerveSubsystem);
    }

        @Override
    public void initialize() {
        super.initialize();

        Pose2d start = swerveSubsystem.getLocalizationPose();
        start = new Pose2d(start.getTranslation(), MathHelper.getAngleAtoB(start, goal.get()));
        Pose2d end = new Pose2d(goal.get().getTranslation(), MathHelper.getAngleAtoB(start, goal.get()));

        List<Waypoint> points = PathPlannerPath.waypointsFromPoses(start, end);

        PathPlannerPath path = new PathPlannerPath(
                points,
                constraints,
                null,
                new GoalEndState(endVel, goal.get().getRotation()));

        pathCommand = AutoBuilder.followPath(path);

        pathCommand.initialize();

        Constants.cXbox.setRumble(GenericHID.RumbleType.kBothRumble, Constants.rumbleHigh);

        System.out.println("goal target: " + goal);
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
