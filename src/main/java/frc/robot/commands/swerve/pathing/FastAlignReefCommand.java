package frc.robot.commands.swerve.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.PID;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class FastAlignReefCommand extends LoggedCommand {

    private SwerveSubsystem s;

    private Supplier<Pose2d> goal;

    private Timer timer;
    private double timeOut;

    private PID xPID = new PID(3, 0, 0, 1.5, 0.2, 0.03, () -> s.getLocalizationPose().getX());
    private PID yPID = new PID(3, 0, 0, 1.5, 0.2, 0.03, () -> s.getLocalizationPose().getY());
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

//        SmartDashboard.putData("PID/x",xPID);
//        SmartDashboard.putData("PID/y",yPID);
    }

    @Override
    public void execute() {
        super.execute();

        xPID.calculate();
        yPID.calculate();
        turnPID.calculate();

        s.driveFieldOriented(new ChassisSpeeds(xPID.getOutput(), yPID.getOutput(), turnPID.getOutput()));

//        if ((Math.abs(xPID.getOutput()) < 0.05 && Math.abs(yPID.getOutput()) < 0.05 && Math.abs(turnPID.getOutput()) < 0.05) || timer.get() > timeOut) {
//            isFinished = true;
//        }

        if (xPID.getDeadband() > Math.abs(xPID.getError()) && yPID.getDeadband() > Math.abs(yPID.getError())) {
            isFinished = true;
            System.out.println("Fast align ended at target");
        }

        if (timer.get() > timeOut) {
            isFinished = true;
            System.out.println("Fast align timed out");
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
