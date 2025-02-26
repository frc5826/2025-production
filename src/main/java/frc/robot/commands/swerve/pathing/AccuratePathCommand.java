package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.PID;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class AccuratePathCommand extends LoggedCommand {

    private Supplier<Pose2d> goalSupplier;

    private SwerveSubsystem swerveSubsystem;

    private boolean finished;

    private Timer timer;
    private double timeEnd;

    private boolean turn;

    private final PIDConstants drivePID = new PIDConstants(2.5, 0.1, 0.1);
    private final PID pidX = new PID(drivePID, 1, 0.4, 0.01, () -> getPose().getX());
    private final PID pidY = new PID(drivePID, 1, 0.4, 0.01, () -> getPose().getY());
    private final PID pidR = new PID(Constants.Swerve.cTurnPID, Math.PI, 0.01, 0.05, () -> goalSupplier.get().getRotation().minus(swerveSubsystem.getLocalizationPose().getRotation()).getRadians());

    public AccuratePathCommand(Pose2d goal, double timeOut, boolean turn, SwerveSubsystem swerveSubsystem) {
        this(() -> goal, timeOut, turn, swerveSubsystem);
    }

    public AccuratePathCommand(Supplier<Pose2d> goal, double timeOut, boolean turn, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        this.goalSupplier = goal;

        timer = new Timer();
        timeEnd = timeOut;

        this.turn = turn;

        addRequirements(swerveSubsystem);
    }

    private Pose2d getPose() {
        return swerveSubsystem.getLocalizationPose();
    }

    @Override
    public void initialize() {
        super.initialize();

        finished = false;

        if (goalSupplier.get().getTranslation().getDistance(swerveSubsystem.getLocalizationPose().getTranslation()) >= 1) {
            System.err.println("AccuratePathCommand - cannot accept values further than 1m from current pose");
            finished = true;
        }

        pidX.setGoal(goalSupplier.get().getX());
        pidY.setGoal(goalSupplier.get().getY());
        pidR.setGoal(0);

        timer.restart();
    }

    @Override
    public void execute() {
        pidX.calculate();
        pidY.calculate();
        pidR.calculate();

        System.out.println("goal: " + goalSupplier.get());
        System.out.println("current pose: " + getPose());

        ChassisSpeeds speeds;

        if (turn) {
            speeds = new ChassisSpeeds(pidX.getOutput(), pidY.getOutput(), pidR.getOutput());
        } else {
            speeds = new ChassisSpeeds(pidX.getOutput(), pidY.getOutput(), 0);
        }

        swerveSubsystem.driveFieldOriented(speeds);

        if (Math.abs(pidX.getError()) <= pidX.getDeadband() &&
                Math.abs(pidY.getError()) <= pidY.getDeadband() &&
                Math.abs(pidR.getError()) <= pidR.getDeadband() ||
                timer.get() > timeEnd) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(0, 0, 0));

        timer.reset();
        finished = false;
    }
}
