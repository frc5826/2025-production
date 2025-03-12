package frc.robot.sensors;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycle;


public class LidarPWM implements NTSendable {

    public DutyCycle lidarReading;
    public DigitalOutput lidarTrigger;

    public LidarPWM(int triggerChannel, int readingChannel){

        lidarTrigger = new DigitalOutput(triggerChannel);
        lidarReading = new DutyCycle(new DigitalInput(readingChannel));

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

    public boolean isOn(){
        return !lidarTrigger.get();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("5826-LidarPWM");
        builder.addDoubleProperty("Measurment M", this::getMeasurement, null);
        builder.addBooleanProperty("Is Sensor On?", this::isOn, null);

    }
}
