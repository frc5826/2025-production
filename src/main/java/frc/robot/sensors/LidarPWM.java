package frc.robot.sensors;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycle;


public class LidarPWM implements NTSendable {

    public DutyCycle lidarReading;
    public DigitalOutput lidarTrigger;
    public String name;

    public LidarPWM(int triggerChannel, int readingChannel, String name){
        lidarTrigger = new DigitalOutput(triggerChannel);
        lidarReading = new DutyCycle(new DigitalInput(readingChannel));
        this.name = name;
    }

    public LidarPWM(int triggerChannel, int readingChannel){
        this(triggerChannel, readingChannel, "");
    }

    public void turnOn(){
        lidarTrigger.set(false);
    }

    public void turnOff(){
        lidarTrigger.set(true);
    }

    /**
     * @return Measurement in meters
     */
    public double getMeasurement() {
        return lidarReading.getHighTimeNanoseconds()/1000000.0;
    }

    //TODO - I'm not sure if this is how this works.
    public boolean isOn(){
        return !lidarTrigger.get();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("5826-LidarPWM");
        builder.addDoubleProperty(name + "/Measurement", this::getMeasurement, null);
        builder.addBooleanProperty(name + "/SensorOn", this::isOn, null);
    }

    public String getName() {
        return name;
    }
}
