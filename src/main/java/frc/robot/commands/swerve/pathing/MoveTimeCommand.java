package frc.robot.commands.swerve.pathing;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveTimeCommand extends LoggedCommand {

    private double moveSec;

    private SwerveSubsystem swerveSubsystem;

    private final Timer timer;

    private final ChassisSpeeds speeds;

    private boolean robotOriented;

    public MoveTimeCommand(double moveSec, ChassisSpeeds speeds, boolean robotOriented,
                           SwerveSubsystem swerveSubsystem) {

        this.moveSec = moveSec;

        this.speeds = speeds;

        this.robotOriented = robotOriented;

        this.swerveSubsystem = swerveSubsystem;

        this.timer = new Timer();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.get() <= moveSec) {
            if (robotOriented) {
                swerveSubsystem.driveRobotOriented(speeds);
            } else {
                swerveSubsystem.driveFieldOriented(speeds);
            }

        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= moveSec) {
            return true;
        } else {
            return super.isFinished();
        }
    }
}
