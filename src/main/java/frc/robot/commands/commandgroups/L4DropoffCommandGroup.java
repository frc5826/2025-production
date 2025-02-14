package frc.robot.commands.commandgroups;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.elevator.ElevatorRepositionCommand;
import frc.robot.commands.swerve.MoveTimeCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.commands.coralizer.CoralizerIntakeCommand.IntakeDirection.OUT;

public class L4DropoffCommandGroup extends SequentialCommandGroup {

    public L4DropoffCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem){
        addCommands(
                new ElevatorRepositionCommand(elevatorSubsystem, -0.3),
                new CoralizerIntakeCommand(coralizerSubsystem, OUT),
                new MoveTimeCommand(0.75, new ChassisSpeeds(-0.5, 0, 0), true, swerveSubsystem),
                new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem)
        );
    }

}
