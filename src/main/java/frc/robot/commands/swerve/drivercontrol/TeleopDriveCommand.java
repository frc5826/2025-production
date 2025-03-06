package frc.robot.commands.swerve.drivercontrol;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Swerve.*;

public class TeleopDriveCommand extends LoggedCommand {

    private final SwerveSubsystem swerveSubsystem;

    private double mult;

    public TeleopDriveCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        double x = -cXbox.getLeftY();
        double y = -cXbox.getLeftX();
        double t = -cXbox.getRightX();

        x = Math.abs(x) > cTeleDriveDeadband ? x : 0;
        y = Math.abs(y) > cTeleDriveDeadband ? y : 0;
        t = Math.abs(t) > cTeleTurnDeadband ? t : 0;

        mult = swerveSubsystem.getSpeedMultiplier();

        ChassisSpeeds speeds = new ChassisSpeeds(
                x * cMaxVelocity * mult, y * cMaxVelocity * mult, t * cMaxAngularVelocity * mult);

        swerveSubsystem.teleDriveFieldOriented(speeds);
    }

}
