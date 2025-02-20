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

public class AccuratePathCommand extends LoggedCommand {

    private Pose2d goal;

    private boolean singleTagAlign;

    private SwerveSubsystem swerveSubsystem;
    private CameraSubsystem cameraSubsystem;

    private boolean finished;

    private Timer timer;
    private double timeEnd;

    private final PIDConstants drivePID = new PIDConstants(2.5, 0.1, 0.1);
    private final PID pidX = new PID(drivePID, 1, 0.03, 0.01, () -> getPose().getX());
    private final PID pidY = new PID(drivePID, 1, 0.03, 0.01, () -> getPose().getY());
    private final PID pidR = new PID(Constants.Swerve.cTurnPID, Math.PI, 0.01, 0.05, () -> swerveSubsystem.getLocalizationPose().getRotation().getRadians());

    public AccuratePathCommand(Pose2d goal, boolean singleTagAlign, CameraSubsystem cameraSubsystem, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.cameraSubsystem = cameraSubsystem;

        this.goal = goal;

        this.singleTagAlign = singleTagAlign;

        timer = new Timer();
        timeEnd = 2;

        addRequirements(swerveSubsystem);
    }

    private Pose2d getPose() {
        if (!singleTagAlign) {
            return swerveSubsystem.getLocalizationPose();
        } else {
            return swerveSubsystem.getLocalizationPose(); //TODO camera only
        }
    }

    @Override
    public void initialize() {
        super.initialize();

        finished = false;

        if (goal.getTranslation().getDistance(swerveSubsystem.getLocalizationPose().getTranslation()) >= 1) {
            System.err.println("AccuratePathCommand - cannot accept values further than 1m from current pose");
            finished = true;
        }

        pidX.setGoal(goal.getX());
        pidY.setGoal(goal.getY());
        pidR.setGoal(goal.getRotation().getRadians());

        timer.restart();
    }

    @Override
    public void execute() {
        pidX.calculate();
        pidY.calculate();
        pidR.calculate();

        ChassisSpeeds speeds = new ChassisSpeeds(pidX.getOutput(), pidY.getOutput(), pidR.getOutput());
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

        timer.reset();
        finished = false;
    }
}
