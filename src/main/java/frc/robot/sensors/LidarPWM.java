package frc.robot.sensors;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycle;


public class LidarPWM implements NTSendable {

    private DutyCycle lidarReading;
    private DigitalOutput lidarTrigger;
    private String name;
    private double offset;

    public LidarPWM(int triggerChannel, int readingChannel, String name, double offset){
        lidarTrigger = new DigitalOutput(triggerChannel);
        lidarReading = new DutyCycle(new DigitalInput(readingChannel));
        this.name = name;
        this.offset = offset;
    }

    public LidarPWM(int triggerChannel, int readingChannel, String name) {
        this(triggerChannel,readingChannel,name,0);
    }

    public LidarPWM(int triggerChannel, int readingChannel){
        this(triggerChannel, readingChannel, "", 0);
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
        return lidarReading.getHighTimeNanoseconds()/1000000.0 + offset;
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
