package frc.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class Dumb extends SequentialCommandGroup {
    public Dumb(SwerveSubsystem s) {

        addCommands(
                new MoveTimeCommand(2.5,
                        new ChassisSpeeds(1, 0, 0),
                        true, s)
        );

    }
}
