package frc.robot.subsystems;

import frc.robot.sensors.LidarPWM;
import frc.robot.Constants.*;


public class DistanceSubsystem extends LoggedSubsystem {

    private double lidar0AllowedVariability;
    private boolean lidarRight0EqualToLeft0;
    //Yes, these are crappy names
    private LidarPWM lidarPWMRight60, lidarPWMLeft60, lidarPWMRight0, lidarPWMLeft0;

    public DistanceSubsystem(){
        //TODO Get correct channels

        this.lidar0AllowedVariability = 0.0;
        this.lidarPWMRight60 = new LidarPWM(3,2);
        this.lidarPWMLeft60 = new LidarPWM(5,4);
        this.lidarPWMRight0 = new LidarPWM(6, 7);
        this.lidarPWMLeft0 = new LidarPWM(8, 9);

    }


    @Override
    public void periodic() {
        super.periodic();

        this.lidarRight0EqualToLeft0 = lidarPWMRight0.getFromBumperMeasurement() == Math.clamp(lidarPWMLeft0.getFromBumperMeasurement(), lidarPWMRight0.getFromBumperMeasurement() - lidar0AllowedVariability, lidarPWMRight0.getFromBumperMeasurement() + lidar0AllowedVariability);
    }

    public boolean angledLidarHitLeft(){
        return lidarPWMLeft60.getFromBumperMeasurement() < 0.01;
    }

    public boolean angledLidarHitRight(){
        return lidarPWMRight60.getFromBumperMeasurement() < 0.01;
    }

    public double angledLidarLeftDistance(){
        return lidarPWMLeft60.getFromBumperMeasurement();
    }

    public double angledLidarRightDistance(){
        return lidarPWMRight60.getFromBumperMeasurement();
    }

    public void enableLidar(){
        lidarPWMLeft60.turnOn();
        lidarPWMRight60.turnOn();
        lidarPWMRight0.turnOn();
        lidarPWMLeft0.turnOn();
    }

    public void disableLidar(){
        lidarPWMLeft60.turnOff();
        lidarPWMRight60.turnOff();
        lidarPWMLeft0.turnOff();
        lidarPWMRight0.turnOff();
    }

    public boolean lidarLeft0IsCloser() {
        return lidarPWMLeft0.getFromBumperMeasurement() < lidarPWMRight0.getFromBumperMeasurement();
    }

//    public boolean lidarRight0IsCloser(){
//        return lidarPWMLeft0.getFromBumperMeasurement() > lidarPWMRight0.getFromBumperMeasurement();
//    }

    public double getDistanceFromReef(){
        double lidarDifference = getLidarDifference();
        if (lidarLeft0IsCloser()){
            return lidarPWMLeft0.getFromBumperMeasurement() - lidarDifference/2;
        }
        else  {
            return lidarPWMRight0.getFromBumperMeasurement() - lidarDifference/2;
        }
    }

    public double getLidarDifference(){
        return Math.abs(lidarPWMLeft0.getFromBumperMeasurement() - lidarPWMRight0.getFromBumperMeasurement());
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

}
