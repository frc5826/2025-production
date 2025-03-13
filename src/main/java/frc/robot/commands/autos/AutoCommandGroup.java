package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandgroups.AlignSourceCommandGroup;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import static frc.robot.positioning.FieldOrientation.getOrientation;

public class AutoCommandGroup extends SequentialCommandGroup {
//
//    PathConstraints slowConstraints = new PathConstraints(1, 1, Math.PI * 1.5, Math.PI * 2);
//    PathConstraints fastConstraints = new PathConstraints(2.5, 3, Math.PI * 3, Math.PI * 4);
//
//
//    public AutoCommandGroup(ElevatorSubsystem e, CoralizerSubsystem c, SwerveSubsystem s, Supplier<Pose2d> station, List<ReefPosition> reefPositions, boolean finishCoral){
//        if (getOrientation().isValid() && !reefPositions.isEmpty()){
//            addCommands(new OnePiece(reefPositions.get(0), station.get(), s, e, c));
//            reefPositions.remove(0);
//            for (ReefPosition reefPosition : reefPositions){
//                addCommands(
//                        new AlignSourceCommandGroup(station, fastConstraints, s, e, c),
//                        new OnePiece(reefPosition, station.get(), s, e, c)
//                );
//            }
//            if(finishCoral) {
//                addCommands(new AlignSourceCommandGroup(station, fastConstraints, s, e, c));
//            }
//        }
//    }

}
