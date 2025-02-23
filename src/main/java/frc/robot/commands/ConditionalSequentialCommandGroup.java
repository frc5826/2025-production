//package frc.robot.commands;
//
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//import java.util.function.BooleanSupplier;
//
//public class ConditionalSequentialCommandGroup extends LoggedCommand {
//
//    private List<Trigger> commandList = new ArrayList<>();
//    private int triggerIndex = -1;
//
//    public void addCommands(Trigger... commands){
//        if (triggerIndex != -1) {
//            throw new IllegalStateException("Commands cannot be added to a composition while it's running");
//        }
//        for (Trigger trigger : commands){
//            Trigger t = trigger.and(() -> triggerIndex >= commandList.indexOf(trigger));
//            commandList.add(trigger);
//        }
//    }
//
//    public void addCommand(Command command, BooleanSupplier condition){
//        if (triggerIndex != -1) {
//            throw new IllegalStateException("Commands cannot be added to a composition while it's running");
//        }
//        Trigger trigger = new Trigger(condition).onTrue(command);
//        trigger.and(() -> triggerIndex >= commandList.indexOf(trigger));
//        commandList.add();
//    }
//
//    public boolean isTriggerReady(){
//
//    }
//
//    @Override
//    public void initialize() {
//        super.initialize();
//        triggerIndex = 0;
//    }
//}