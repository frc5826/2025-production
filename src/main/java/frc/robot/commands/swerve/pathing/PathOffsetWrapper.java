package frc.robot.commands.swerve.pathing;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandWrapper;
import frc.robot.positioning.FieldOrientation;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.cButtonBoard;

public class PathOffsetWrapper extends CommandWrapper {

    private Supplier<Command> commandSupplier;

    public PathOffsetWrapper(Supplier<Pose2d> pose2d, PathConstraints constraints, double offset, boolean reef, SwerveSubsystem swerveSubsystem){
        commandSupplier =  () -> new PathOffsetThenAccurateCommand(pose2d.get(), constraints, offset, reef, swerveSubsystem);
        addRequirements(commandSupplier.get().getRequirements());
    }

    @Override
    public Command setCommand() {
        return commandSupplier.get();

    }
}
