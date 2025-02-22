package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

public class ConditionalSequentialCommandGroup extends LoggedCommand {
    private final List<ConditionallyActivatedCommand> commands = new ArrayList<>();
    private List<ConditionallyActivatedCommand> runningCommands;
    private int currentCommandIndex = -1;

    public void addCommands(ConditionallyActivatedCommand... commands){
        if (currentCommandIndex != -1) {
            throw new IllegalStateException("Commands cannot be added to a composition while it's running");
        }
        this.commands.addAll(Arrays.asList(commands));
    }

    public void addCommands(Command... commands){
        if (currentCommandIndex != -1) {
            throw new IllegalStateException("Commands cannot be added to a composition while it's running");
        }
        for (Command command : commands){
            this.commands.add(new ConditionallyActivatedCommand(command));
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        runningCommands = new ArrayList<>();
        currentCommandIndex = 0;
        if (!commands.isEmpty()){
            for (ConditionallyActivatedCommand command : commands) { command.markFinished(true); command.markedRan(false);}
            commands.getFirst().getCommand().initialize();
            runningCommands.add(commands.getFirst());
        }
    }

    @Override
    public void execute() {
        super.execute();
        for (ConditionallyActivatedCommand command : runningCommands){
            if(command.isFinished()) {
                command.markFinished(true);
            }
            else {
                command.execute();
            }
        }
        if (currentCommandIndex < commands.size() - 1) {
            if (commands.get(currentCommandIndex + 1).attemptInitialize()) {
                currentCommandIndex++;
                runningCommands.add(commands.get(currentCommandIndex));
            }
        }
    }

    @Override
    public boolean isFinished() {
        boolean finished = true;
        for (ConditionallyActivatedCommand runningCommand : runningCommands) {
            if(!isFinished()){
                finished = false;
            }
        }
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        currentCommandIndex = -1;
    }

    public class ConditionallyActivatedCommand {

        private Command command;
        private BooleanSupplier condition;
        private boolean finished = false;
        private boolean ran = false;

        public ConditionallyActivatedCommand(Command command, BooleanSupplier condition) {
            this.command = command;
            this.condition = condition;
        }

        public ConditionallyActivatedCommand(Command command){
            this.command = command;
            this.condition = () -> true;
        }

        public boolean attemptInitialize(){
            if (condition.getAsBoolean()){
                command.initialize();
                ran = true;
                return true;
            }
            return false;
        }

        public void execute(){
            command.execute();
        }

        public boolean isFinished(){
            if(ran) {
                if (finished) {
                    return true;
                }
                if (command.isFinished() && !finished) {
                    finished = true;
                    end();
                }
                return command.isFinished();
            }
            return false;
        }

        public void end(){
            command.end(false);
        }

        public void markFinished(boolean finished){
            this.finished = finished;
        }

        public void markedRan(boolean ran){
            this.ran = ran;
        }

        public Command getCommand() {
            return command;
        }
    }
}