package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class ButtonBoard {

    private BooleanSubscriber[] buttons;

    public ButtonBoard(int totalButtons) {
        buttons = new BooleanSubscriber[totalButtons];
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        for(int i = 0; i < totalButtons; i++){
            buttons[i] = instance.getBooleanTopic("buttons/" + i).subscribe(false, PubSubOption.hidden(false));
        }
    }

    //TODO Get Button Pressed
    public boolean getButton(int button){
        if (button < buttons.length && button >= 0) {
            return buttons[button].get();
        }
        else {
            System.err.println("Button " + button + " does not exist!");
            return false;
        }
    }

}
