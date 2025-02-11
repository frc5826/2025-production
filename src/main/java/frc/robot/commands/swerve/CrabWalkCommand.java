package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class CrabWalkCommand extends LoggedCommand {

    private SwerveSubsystem swerveSubsystem;
    private boolean right;
    private ChassisSpeeds speeds;
    private double speed;

    public CrabWalkCommand(boolean right, double speed, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.right = right;
        this.speed = speed;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        double team = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? 1 : -1;
        double field = (Math.abs(swerveSubsystem.getLocalizationPose().getRotation().getDegrees()) > 90) ? -1 : 1;
        double robot = (right) ? -1 : 1;

        speeds = new ChassisSpeeds(0, speed * field * robot * team, 0);
    }

    @Override
    public void execute() {
        super.execute();
        swerveSubsystem.driveRobotOriented(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(0, 0, 0));
    }
}
