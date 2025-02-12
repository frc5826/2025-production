package frc.robot.sensors;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

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

    public double getOffsetUltrasonic(){
        if (getUltrasonic() <= 0.30){
            return 0.00;
        }
        else {
            return getUltrasonic() - 0.30;
        }
    }


    @Override
    public void initSendable(NTSendableBuilder builder) {

        builder.setSmartDashboardType("5826-Ultrasonic");
        builder.addDoubleProperty("FromSensorMeters", this::getUltrasonic, null);
        builder.addDoubleProperty("FromBumperMeters", this::getOffsetUltrasonic, null);
    }
}
