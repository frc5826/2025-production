package frc.robot.commands.swerve.drivercontrol;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class CrabWalkCommand extends LoggedCommand {

    private SwerveSubsystem swerveSubsystem;
    private Direction direction;
    private ChassisSpeeds speeds;
    private double speed;

    public CrabWalkCommand(Direction direction, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.direction = direction;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        double rightleft = 0;
        double frontback = 0;

        if (direction == Direction.RIGHT || direction == Direction.LEFT) {
            rightleft = (direction == Direction.RIGHT) ? -1 : 1;
        } else {
            frontback = (direction == Direction.FRONT) ? 1 : -1;
        }

        speeds = new ChassisSpeeds(frontback, rightleft, 0);
    }

    @Override
    public void execute() {
        super.execute();
        speed = swerveSubsystem.getCrabSpeedMult();
        swerveSubsystem.driveRobotOriented(speeds.times(speed));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(0, 0, 0));
    }

    public static enum Direction {
        FRONT,
        BACK,
        RIGHT,
        LEFT
    }
}
