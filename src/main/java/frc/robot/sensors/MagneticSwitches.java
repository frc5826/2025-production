package frc.robot.sensors;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

public class MagneticSwitches implements NTSendable {

    private DigitalInput limitSwitch1, limitSwitch2;

    public MagneticSwitches(int channel1, int channel2){
        limitSwitch1 = new DigitalInput(channel1);
        limitSwitch2 = new DigitalInput(channel2);
    }

    public boolean getMagSwitch(){
        return limitSwitch1.get() && limitSwitch2.get();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("5826-MagSwitches");
        builder.addBooleanProperty("All Switched?", this::getMagSwitch, null);
    }
}
