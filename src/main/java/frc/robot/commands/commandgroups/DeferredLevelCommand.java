package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LoggedCommand;
import frc.robot.commands.commandgroups.reef.L1CommandGroup;
import frc.robot.commands.commandgroups.reef.L2CommandGroup;
import frc.robot.commands.commandgroups.reef.L3CommandGroup;
import frc.robot.commands.commandgroups.reef.L4CommandGroup;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DeferredLevelCommand extends LoggedCommand {

    private final L1CommandGroup l1CommandGroup;
    private final L2CommandGroup l2CommandGroup;
    private final L3CommandGroup l3CommandGroup;
    private final L4CommandGroup l4CommandGroup;

    private DeferredLevel target;
    private Command running;

    public DeferredLevelCommand(ElevatorSubsystem e, CoralizerSubsystem c, SwerveSubsystem s) {
        l1CommandGroup = new L1CommandGroup(e, c);
        l2CommandGroup = new L2CommandGroup(e, c);
        l3CommandGroup = new L3CommandGroup(e, c);
        l4CommandGroup = new L4CommandGroup(e, c, s);
        target = DeferredLevel.NONE;
    }

    @Override
    public void initialize() {
        super.initialize();
        switch (target){
            case L1 -> running = l1CommandGroup;
            case L2 -> running = l2CommandGroup;
            case L3 -> running = l3CommandGroup;
            case L4 -> running = l4CommandGroup;
            default -> {}
        }
        if (target != DeferredLevel.NONE) {
            running.initialize();
        }
    }

    @Override
    public void execute() {
        super.execute();
        running.execute();
    }

    @Override
    public boolean isFinished() {
        return running.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        running.end(interrupted);
    }

    public void setTarget(DeferredLevel target) {
        this.target = target;
    }

    public static enum DeferredLevel {
        NONE,
        L1,
        L2,
        L3,
        L4
    }
}
