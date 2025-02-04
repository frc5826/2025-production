package frc.robot.sensors;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

//TODO
public class UltrasonicPWM implements NTSendable {


    private DutyCycle ultrasonicReading;


    public UltrasonicPWM(int channel){
        ultrasonicReading = new DutyCycle(new DigitalInput(channel));
    }

    /**
     * @return Reading in Meters
     */
    public double getUltrasonic(){
        return ultrasonicReading.getHighTimeNanoseconds()/100000.0;
    }


    @Override
    public void initSendable(NTSendableBuilder builder) {

        builder.setSmartDashboardType("5826-Ultrasonic");
        builder.addDoubleProperty("Millimeters", this::getUltrasonic, null);
    }
}
