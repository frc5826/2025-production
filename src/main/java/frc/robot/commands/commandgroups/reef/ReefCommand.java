package frc.robot.commands.commandgroups.reef;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ReefTargeting;

public class ReefCommand extends Command {

    private ElevatorSubsystem e;
    private CoralizerSubsystem c;
    private ReefTargeting reefTargeting;

    private Command command;

    public ReefCommand(ReefTargeting reefTargeting, ElevatorSubsystem elevatorSubsystem, CoralizerSubsystem coralizerSubsystem) {
        this.reefTargeting = reefTargeting;
        this.e = elevatorSubsystem;
        this.c = coralizerSubsystem;

        addRequirements(e,c);
    }

    @Override
    public void initialize() {
        super.initialize();

        command = Commands.sequence(
                new ElevatorPositionCommand(e, reefTargeting.getLevel().get().height, reefTargeting.getLevel().get()),
                new CoralizerWristCommand(c, reefTargeting.getLevel().get().angle)
        );

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
        super.end(interrupted);
        command.end(interrupted);
    }
}
