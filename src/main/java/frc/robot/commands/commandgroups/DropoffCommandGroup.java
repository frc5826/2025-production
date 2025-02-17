package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.commands.coralizer.CoralizerIntakeCommand.IntakeDirection.OUT;
import static frc.robot.subsystems.ElevatorSubsystem.LevelTarget.*;

public class DropoffCommandGroup extends Command {

    private Command command;
    private ElevatorSubsystem elevatorSubsystem;
    private CoralizerSubsystem coralizerSubsystem;
    private SwerveSubsystem swerveSubsystem;

    public DropoffCommandGroup(ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem, SwerveSubsystem swerveSubsystem){

         this.elevatorSubsystem = elevatorSubsystem;
         this.coralizerSubsystem = coralizerSubsystem;
         this.swerveSubsystem = swerveSubsystem;
         addRequirements(elevatorSubsystem, swerveSubsystem, coralizerSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    ElevatorSubsystem.LevelTarget levelTarget = elevatorSubsystem.getLevelTarget();
        if (levelTarget == L1){
            command = new L1DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem);
        }
        else if (levelTarget == L2 || levelTarget == L3){
            command = new L3L2DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem);
        }
        else if (levelTarget == L4){
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
