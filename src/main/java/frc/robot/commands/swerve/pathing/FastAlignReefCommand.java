package frc.robot.commands.swerve.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.math.PID;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class FastAlignReefCommand extends Command {

    private SwerveSubsystem s;

    private Supplier<Pose2d> goal;

    private Timer timer;
    private double timeOut;

    private PID xPID = new PID(3, 0, 0, 1.5, 0.1, 0.01, () -> s.getLocalizationPose().getX());
    private PID yPID = new PID(3, 0, 0, 1.5, 0.1, 0.01, () -> s.getLocalizationPose().getY());
    private PID turnPID = new PID(Constants.Swerve.cTurnPID, 4, 0, Math.toRadians(3), this::angleDiff);

    private boolean isFinished;

    //TODO test this thang
    public FastAlignReefCommand(Supplier<Pose2d> goal, double timeOut, SwerveSubsystem swerveSubsystem) {
        this.s = swerveSubsystem;
        this.goal = goal;
        this.timeOut = timeOut;

        timer = new Timer();

        isFinished = false;

        addRequirements(s);
    }

    @Override
    public void initialize() {
        super.initialize();

        xPID.setGoal(goal.get().getX());
        yPID.setGoal(goal.get().getY());
        turnPID.setGoal(0);

        timer.restart();
    }

    @Override
    public void execute() {
        super.execute();

        xPID.calculate();
        yPID.calculate();
        turnPID.calculate();

        s.driveFieldOriented(new ChassisSpeeds(xPID.getOutput(), yPID.getOutput(), turnPID.getOutput()));

        if ((xPID.getOutput() == 0 && yPID.getOutput() == 0 && turnPID.getOutput() == 0) || timer.get() > timeOut) {
            isFinished = true;
        }
    }

    private double angleDiff() {
        return s.getLocalizationPose().getRotation().minus(goal.get().getRotation()).getRadians();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        timer.reset();
        isFinished = false;
    }
}
