package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.sensors.LidarPWM;
//import frc.robot.sensors.UltrasonicAN;


public class DistanceSubsystem extends LoggedSubsystem {

//    private UltrasonicAN ultrasonic1, ultrasonic2;
    private double ultrasonicAllowedVariability;
    private boolean ultrasonic1EqualTo2;
    private boolean ultrasonic2EqualTo1;
    private LidarPWM lidarPWMRight, lidarPWMLeft;

    public DistanceSubsystem(){
        //TODO Get correct channels
//        this.ultrasonic2 = new UltrasonicAN(1, 0);
//        this.ultrasonic1 = new UltrasonicAN(2, 3);
//        this.ultrasonicAllowedVariability = 0.0;

        this.lidarPWMRight = new LidarPWM(4,5);
        this.lidarPWMLeft = new LidarPWM(6,7);

    }

    public boolean lidarHitLeft(){
        return lidarPWMLeft.getMeasurement() < 0.1;
    }

    public boolean lidarHitRight(){
        return lidarPWMRight.getMeasurement() < 0.1;
    }
//
//    @Override
//    public void periodic() {
//        super.periodic();

//        this.ultrasonic2EqualTo1 = ultrasonic2.getUltrasonicFromBumper() == Math.clamp(ultrasonic1.getUltrasonicFromBumper(), ultrasonic2.getUltrasonicFromBumper() - ultrasonicAllowedVariability, ultrasonic2.getUltrasonicFromBumper() + ultrasonicAllowedVariability);
//        this.ultrasonic1EqualTo2 = ultrasonic1.getUltrasonicFromBumper() == Math.clamp(ultrasonic2.getUltrasonicFromBumper(), ultrasonic1.getUltrasonicFromBumper() - ultrasonicAllowedVariability, ultrasonic1.getUltrasonicFromBumper() + ultrasonicAllowedVariability);
    //}


//    public double getUltrasonicDifference(){
////        return Math.abs(ultrasonic1.getUltrasonicFromBumper() - ultrasonic2.getUltrasonicFromBumper());
//    }
//
//    public boolean ultrasonicsDontEqual(){
////        return !ultrasonic1EqualTo2 || !ultrasonic2EqualTo1;
//    }
//
//    public double getDistanceFromReef(){
////        double ultrasonicDifference = getUltrasonicDifference();
////        if (ultrasonic1.getUltrasonicFromBumper() > ultrasonic2.getUltrasonicFromBumper()){
////            return ultrasonic1.getUltrasonicFromBumper() - ultrasonicDifference/2;
////        } else if (ultrasonic1.getUltrasonicFromBumper() < ultrasonic2.getUltrasonicFromBumper()) {
////            return ultrasonic2.getUltrasonicFromBumper() - ultrasonicDifference/2;
////        }
////        return 0.0;
//    }

//    public double getTurnAngle(){
//        double ultrasonicDifference = getUltrasonicDifference();
//        return Math.toDegrees(Math.atan2(ultrasonicDifference, Constants.BluePositions.cRobotWidth));
//    }
}
