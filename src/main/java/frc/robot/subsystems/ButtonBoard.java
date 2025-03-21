package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonBoard {


    private BooleanSubscriber[] buttons;
    private boolean[] buttonPressed;
    private StringLogEntry buttonLog;

    public ButtonBoard(int totalButtons) {
        buttonPressed = new boolean[totalButtons];
        buttons = new BooleanSubscriber[totalButtons];
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        if(!DataLogManager.getLogDir().isEmpty())
            buttonLog = new StringLogEntry(DataLogManager.getLog(),"NT/buttons/log");
        for (int i = 0; i < totalButtons; i++) {
            buttons[i] = instance.getBooleanTopic("buttons/" + i).subscribe(false, PubSubOption.hidden(false));
        }
    }

    public boolean getButton(int button) {
        if (button < buttons.length && button >= 0) {
            boolean buttonValue = buttons[button].get();
            if (!buttonPressed[button] && buttonValue) {
                buttonLog.append("Button " + button + " was pressed");
                buttonPressed[button] = true;
            } else
                buttonPressed[button] = buttonValue;
            return buttonValue;
        } else {
            System.err.println("Button " + button + " does not exist!");
            return false;
        }
    }

    public boolean getButtonPressed(int button) {
        if (button < buttons.length && button >= 0) {
            boolean buttonValue = buttons[button].get();
            if (buttonPressed[button]) {
                buttonPressed[button] = buttonValue;
                return false;
            } else {
                buttonLog.append("Button " + button + " was pressed");
                buttonPressed[button] = buttonValue;
                return buttonValue;
            }
        } else {
            System.err.println("Button " + button + " does not exist!");
            return false;
        }
    }

    public boolean getButtonReleased(int button) {
        if (button < buttons.length && button >= 0) {
            boolean buttonValue = buttons[button].get();
            if (buttonPressed[button]) {
                buttonPressed[button] = buttonValue;
                return !buttonValue;
            } else {
                buttonPressed[button] = buttonValue;
                return false;
            }
        } else {
            System.err.println("Button " + button + " does not exist!");
            return false;
        }
    }

}
