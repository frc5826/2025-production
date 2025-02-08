package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class Right extends SequentialCommandGroup {

    private SwerveSubsystem s;

    private PathConstraints slowConstraints;
    private PathConstraints

    public Right(SwerveSubsystem swerveSubsystem) {
        this.s = swerveSubsystem;

    }

}
