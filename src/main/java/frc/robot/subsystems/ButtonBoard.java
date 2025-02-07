package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonBoard extends SubsystemBase {


    private BooleanSubscriber[] buttons;
    private boolean[] buttonPressed;
    private BooleanPublisher connectionPublisher;

    public ButtonBoard(int totalButtons) {
        buttonPressed = new boolean[totalButtons];
        buttons = new BooleanSubscriber[totalButtons];
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        connectionPublisher = instance.getBooleanTopic("buttons/isConnected").publish();
        for (int i = 0; i < totalButtons; i++) {
            buttons[i] = instance.getBooleanTopic("buttons/" + i).subscribe(false, PubSubOption.hidden(false));
        }
    }

    @Override
    public void periodic() {
        boolean isConnected = false;
        for (ConnectionInfo c:NetworkTableInstance.getDefault().getConnections()){
            if (c.remote_id.equals("button-client")) {
                isConnected = true;
                break;
            }
        }
        connectionPublisher.set(isConnected);
    }

    public boolean getButton(int button) {
        if (button < buttons.length && button >= 0) {
            return buttons[button].get();
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
