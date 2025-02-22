package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class CommandWrapper extends Command {

    private Command command;

    public abstract Command setCommand();

    @Override
    public void initialize() {
        command = setCommand();
        addRequirements(command.getRequirements());
        command.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void execute() {
        command.execute();
    }
}
