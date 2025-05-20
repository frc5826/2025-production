package frc.robot.commands.commandgroups.dropoff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.*;

import static frc.robot.commands.coralizer.CoralizerIntakeCommand.IntakeDirection.OUT;

public class DropoffCommandGroup extends Command {

    private Command command;
    private ElevatorSubsystem elevatorSubsystem;
    private ReefTargeting reefTargeting;
    private CoralizerSubsystem coralizerSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public DropoffCommandGroup(ReefTargeting reefTargeting, ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, ShooterSubsystem shooterSubsystem){

         this.elevatorSubsystem = elevatorSubsystem;
         this.reefTargeting = reefTargeting;
         this.coralizerSubsystem = coralizerSubsystem;
         addRequirements(elevatorSubsystem, coralizerSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    ReefPosition.ReefLevel levelTarget = reefTargeting.getLevel().get();
        if (levelTarget.equals(ReefPosition.ReefLevel.L1)){
            command = new L1DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, shooterSubsystem);
        }
        else if (levelTarget.equals(ReefPosition.ReefLevel.L2) || levelTarget.equals(ReefPosition.ReefLevel.L3)){
            command = new L3L2DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, shooterSubsystem);
        }
        else if (levelTarget.equals(ReefPosition.ReefLevel.L4)){
            command = new L4DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, shooterSubsystem);
        }
        else {
            command = new CoralizerIntakeCommand(shooterSubsystem, OUT);
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
