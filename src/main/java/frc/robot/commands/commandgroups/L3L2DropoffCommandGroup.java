package frc.robot.commands.commandgroups;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorRepositionCommand;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class L3L2DropoffCommandGroup extends SequentialCommandGroup {

    public L3L2DropoffCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem){
        addCommands(
                new ElevatorRepositionCommand(elevatorSubsystem, -0.3),
                //new CoralizerWristCommand(coralizerSubsystem, 10),
                new MoveTimeCommand(0.5, new ChassisSpeeds(-0.5, 0, 0), true, swerveSubsystem),
                new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem)
        );
    }

}
