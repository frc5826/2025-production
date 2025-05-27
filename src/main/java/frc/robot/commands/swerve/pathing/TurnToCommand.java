package frc.robot.commands.swerve.pathing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.math.PID;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Swerve.cMaxAngularVelocity;
import static frc.robot.Constants.Swerve.cMaxVelocity;

public class TurnToCommand extends Command {

    private SwerveSubsystem s;

    private Rotation2d goal;

    private PID pid = new PID(Swerve.cTurnPID, Math.PI, 0, Math.toRadians(3), this::angleToTurn);

    public TurnToCommand(Rotation2d goal, SwerveSubsystem swerveSubsystem) {
        this.s = swerveSubsystem;
        this.goal = goal;

        addRequirements(s);
    }

    @Override
    public void initialize() {
        super.initialize();

        pid.setGoal(0);
    }

    @Override
    public void execute() {
        super.execute();

        double x = -cXbox.getLeftY();
        double y = -cXbox.getLeftX();

        x = Math.abs(x) > cTeleDriveDeadband ? x : 0;
        y = Math.abs(y) > cTeleDriveDeadband ? y : 0;

        pid.calculate();
        double rotation = pid.getOutput();

        ChassisSpeeds speeds = new ChassisSpeeds(x * cMaxVelocity * s.getSpeedMultiplier(), y * cMaxVelocity * s.getSpeedMultiplier(), rotation);

        s.teleDriveFieldOriented(speeds);
    }

    private double angleToTurn() {
        return s.getLocalizationPose().getRotation().minus(goal).getRadians();
    }

}
