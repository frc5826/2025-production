package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.LidarPWM;
import frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class DistanceSubsystem extends LoggedSubsystem {

    private double lidar0AllowedVariability;
    private boolean lidarRight0EqualToLeft0;
    //Yes, these are crappy names
    private LidarPWM lidarPWMRight60, lidarPWMRight0;
    private ReadingBuffer lidarPWMRight60Buffer, lidarPWMRight0Buffer;
    //private LidarPWM lidarPWMLeft60, lidarPWMLeft0;


    public DistanceSubsystem(){
        //TODO Get correct channels

        this.lidar0AllowedVariability = 0.0;
        this.lidarPWMRight60 = new LidarPWM(7,6);
        //this.lidarPWMLeft60 = new LidarPWM(5,4);
        this.lidarPWMRight0 = new LidarPWM(3, 2);
        //this.lidarPWMLeft0 = new LidarPWM(8, 9);
        this.lidarPWMRight60Buffer = new ReadingBuffer(9);
        this.lidarPWMRight0Buffer = new ReadingBuffer(9);

    }


    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("lidars/AngledHitRight", angledLidarHitRight());
        SmartDashboard.putBoolean("lidars/TouchingWall", isTouching());
        SmartDashboard.putNumber("lidars/AngledRightDistance", angledLidarRightDistance());
        SmartDashboard.putNumber("lidars/DistanceFromReef", getDistanceFromReef());
        SmartDashboard.putNumber("lidars/Skew", getSkew());

        if(lidarPWMRight60.isOn()){
            lidarPWMRight60Buffer.add(lidarPWMRight60.getMeasurement());
        }
        else {
            lidarPWMRight60Buffer.clear();
        }
        if(lidarPWMRight0.isOn()){
            lidarPWMRight0Buffer.add(lidarPWMRight0.getMeasurement());
        }
        else {
            lidarPWMRight0Buffer.clear();
        }


        this.lidarRight0EqualToLeft0 = true;//lidarPWMRight0.getFromBumperMeasurement() == Math.clamp(lidarPWMLeft0.getFromBumperMeasurement(), lidarPWMRight0.getFromBumperMeasurement() - lidar0AllowedVariability, lidarPWMRight0.getFromBumperMeasurement() + lidar0AllowedVariability);
    }

    private double getFromBumperMeasurement(ReadingBuffer buffer){
        if (buffer.getMedian() <= 0.58){
            return 0.00;
        }
        else {
            //TODO get real distance from bumper to Lidar
            //This should be the distance (In meters) that the Lidar is from the edge of the front bumper
            return buffer.getMedian() - 0.56;
        }

    }

    public boolean angledLidarHitLeft(){
        return false;//lidarPWMLeft60.getFromBumperMeasurement() < 0.01;
    }

    public boolean angledLidarHitRight(){
        return getFromBumperMeasurement(lidarPWMRight60Buffer) < 0.23;
    }

    public double angledLidarLeftDistance(){
        return 0.0; //lidarPWMLeft60.getFromBumperMeasurement();
    }

    public double angledLidarRightDistance(){
        return getFromBumperMeasurement(lidarPWMRight60Buffer);
    }

    public void enableLidar(){
        //lidarPWMLeft60.turnOn();
        lidarPWMRight60.turnOn();
        lidarPWMRight0.turnOn();
        //lidarPWMLeft0.turnOn();
    }

    public void disableLidar(){
        //lidarPWMLeft60.turnOff();
        lidarPWMRight60.turnOff();
        //lidarPWMLeft0.turnOff();
        lidarPWMRight0.turnOff();
    }

    public boolean lidarLeft0IsCloser() {
        return false; //lidarPWMLeft0.getFromBumperMeasurement() < lidarPWMRight0.getFromBumperMeasurement();
    }

//    public boolean lidarRight0IsCloser(){
//        return lidarPWMLeft0.getFromBumperMeasurement() > lidarPWMRight0.getFromBumperMeasurement();
//    }

    public double getDistanceFromReef(){
        double lidarDifference = getLidarDifference();
        if (lidarLeft0IsCloser()){
            return 0.0; //lidarPWMLeft0.getFromBumperMeasurement() - lidarDifference/2;
        }
        else  {
            return getFromBumperMeasurement(lidarPWMRight0Buffer)  - lidarDifference/2;
        }
    }

    public double getLidarDifference(){
        return 0.0; //Math.abs(lidarPWMLeft0.getFromBumperMeasurement() - lidarPWMRight0.getFromBumperMeasurement());
    }

    public boolean lidars0DontEqual(){
        return !lidarRight0EqualToLeft0;
    }

    public boolean isTouching() {
        return !lidars0DontEqual() && getDistanceFromReef() < 0.1;
    }

    public double getSkew() {
        //assume 90 degrees is left and -90 is right
        return lidarLeft0IsCloser() ? getTurnAngle() * -1 : getTurnAngle();
    }

    public double getTurnAngle(){
        double lidarDifference = getLidarDifference();
        return Math.toDegrees(Math.atan2(lidarDifference, BluePositions.cRobotWidth));
    }

    private static class ReadingBuffer{
        private final int bufferSize;
        private final List<Double> buffer;
        private int counter;

        public ReadingBuffer(int bufferSize){

            counter = 0;
            this.bufferSize = bufferSize;
            this.buffer = new ArrayList<>(bufferSize);
            this.clear();

        }

        public void add(double reading){
            buffer.set(counter, reading);
            counter = (counter + 1) % bufferSize;
        }

        public double getMedian(){
            return buffer.stream().sorted().toList().get((int) Math.ceil(bufferSize/2.0));
        }

        public void clear(){
            this.buffer.clear();
            for(int i = 0; i < bufferSize; i++){
                this.buffer.add(0.);
            }
        }

    }


}
