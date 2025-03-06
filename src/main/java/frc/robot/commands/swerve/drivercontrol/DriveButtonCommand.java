package frc.robot.commands.swerve.drivercontrol;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveButtonCommand extends LoggedCommand {

    private SwerveSubsystem s;
    private ChassisSpeeds speeds;

    public DriveButtonCommand(ChassisSpeeds speeds, SwerveSubsystem swerveSubsystem) {
        this.s = swerveSubsystem;
        this.speeds = speeds;

        addRequirements(s);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        s.driveRobotOriented(speeds);
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        s.driveRobotOriented(new ChassisSpeeds(0, 0, 0));
    }
}
