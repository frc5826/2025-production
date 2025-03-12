package frc.robot.commands.commandgroups.dropoff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ReefTargeting;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.commands.coralizer.CoralizerIntakeCommand.IntakeDirection.OUT;

public class DropoffCommandGroup extends Command {

    private Command command;
    private ElevatorSubsystem elevatorSubsystem;
    private ReefTargeting reefTargeting;
    private CoralizerSubsystem coralizerSubsystem;
    private SwerveSubsystem swerveSubsystem;

    public DropoffCommandGroup(ReefTargeting reefTargeting, ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem){

         this.elevatorSubsystem = elevatorSubsystem;
         this.reefTargeting = reefTargeting;
         this.coralizerSubsystem = coralizerSubsystem;
         this.swerveSubsystem = swerveSubsystem;
         addRequirements(elevatorSubsystem, swerveSubsystem, coralizerSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    ReefPosition.ReefLevel levelTarget = reefTargeting.getLevel();
        if (levelTarget.equals(ReefPosition.ReefLevel.L1)){
            command = new L1DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem);
        }
        else if (levelTarget.equals(ReefPosition.ReefLevel.L2) || levelTarget.equals(ReefPosition.ReefLevel.L3)){
            command = new L3L2DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem);
        }
        else if (levelTarget.equals(ReefPosition.ReefLevel.L4)){
            command = new L4DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem);
        }
        else {
            command = new CoralizerIntakeCommand(coralizerSubsystem, OUT);
        }
        command.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}
