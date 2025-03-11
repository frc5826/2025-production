package frc.robot.sensors;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

import java.util.concurrent.TimeUnit;

public class UltrasonicPWM implements NTSendable {


    private DutyCycle ultrasonicReading;


    public UltrasonicPWM(int channel){
        ultrasonicReading = new DutyCycle(new DigitalInput(channel));
    }

    /**
     * @return Reading in Meters
     */
    public double getUltrasonic(){
        return TimeUnit.NANOSECONDS.toSeconds(ultrasonicReading.getHighTimeNanoseconds());
    }

    public double getUltrasonicFromBumper(){
        if (getUltrasonic() <= 0.30){
            return 0.00;
        }
        else {
            //This should be the distance (In meters) that the ultrasonic is from the edge of the front bumper
            return getUltrasonic() - 0.50;
        }
    }


    @Override
    public void initSendable(NTSendableBuilder builder) {

        builder.setSmartDashboardType("5826-Ultrasonic");
        builder.addDoubleProperty("FromSensorMeters", this::getUltrasonic, null);
        builder.addDoubleProperty("FromBumperMeters", this::getUltrasonicFromBumper, null);
    }
}
