package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.math.MathHelper;
import frc.robot.sensors.LidarPWM;
import static frc.robot.Constants.Distance.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;


public class DistanceSubsystem extends LoggedSubsystem {

    private boolean lidarRight0EqualToLeft0;
    private double lidar0AllowedVariability;

    //Yes, these are crappy names
    private final LidarPWM lidarPWMRight60, lidarPWMRight0;
    private final ReadingBuffer lidarPWMRight60Buffer, lidarPWMRight0Buffer;
    private final LidarPWM lidarPWMLeft60, lidarPWMLeft0;
    private final ReadingBuffer lidarPWMLeft60Buffer, lidarPWMLeft0Buffer;
    private final List<LidarPWM> lidars;
    private final List<ReadingBuffer> buffers;

    public DistanceSubsystem(){
        lidar0AllowedVariability = 0.0;

        this.lidarPWMRight60 = new LidarPWM(lidarPWMRight60TriggerPort,lidarPWMRight60ReadingPort, "Right60", cR60Offset);
        this.lidarPWMRight0 = new LidarPWM(lidarPWMRight0TriggerPort,lidarPWMRight0ReadingPort, "Right0", cR0Offset);
        this.lidarPWMLeft0 = new LidarPWM(lidarPWMLeft0TriggerPort,lidarPWMLeft0ReadingPort, "Left0", cL0Offset);
        this.lidarPWMLeft60 = new LidarPWM(lidarPWMLeft60TriggerPort,lidarPWMLeft60ReadingPort, "Left60", cL60Offset);
        this.lidars = List.of(lidarPWMRight60, lidarPWMRight0, lidarPWMLeft0, lidarPWMLeft60);
        this.lidars.forEach(LidarPWM::turnOff);

        this.lidarPWMRight60Buffer = new ReadingBuffer(lidarBufferSize, this.lidarPWMRight60::getMeasurement, this.lidarPWMRight60::isOn);
        this.lidarPWMRight0Buffer = new ReadingBuffer(lidarBufferSize, this.lidarPWMRight0::getMeasurement, this.lidarPWMRight0::isOn);
        this.lidarPWMLeft0Buffer = new ReadingBuffer(lidarBufferSize, this.lidarPWMLeft0::getMeasurement, this.lidarPWMLeft0::isOn);
        this.lidarPWMLeft60Buffer = new ReadingBuffer(lidarBufferSize, this.lidarPWMLeft60::getMeasurement, this.lidarPWMLeft60::isOn);
        this.buffers = List.of(lidarPWMRight60Buffer, lidarPWMRight0Buffer, lidarPWMLeft0Buffer, lidarPWMLeft60Buffer);

    }


    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("lidars/AngledHitRight", angledLidarHitRightClose());
        SmartDashboard.putBoolean("lidars/AngledHitLeft", angledLidarHitLeftClose());

        SmartDashboard.putBoolean("lidars/TouchingWall", isTouching());

        SmartDashboard.putNumber("lidars/AngledRightDistance", angledLidarRightDistance());
        SmartDashboard.putNumber("lidars/AngledLeftDistance", angledLidarLeftDistance());
        
        SmartDashboard.putNumber("lidars/DistanceFromReef", getDistanceFromReef());
        SmartDashboard.putNumber("lidars/Skew", getSkew());

        SmartDashboard.putNumber("lidars/" + lidarPWMRight60.getName() + "-median", lidarPWMRight60Buffer.getMedian());
        SmartDashboard.putNumber("lidars/" + lidarPWMRight0.getName() + "-median", lidarPWMRight0Buffer.getMedian());
        SmartDashboard.putNumber("lidars/" + lidarPWMLeft0.getName() + "-median", lidarPWMLeft0Buffer.getMedian());
        SmartDashboard.putNumber("lidars/" + lidarPWMLeft60.getName() + "-median", lidarPWMLeft60Buffer.getMedian());


        this.buffers.forEach(ReadingBuffer::add);
        this.lidarRight0EqualToLeft0 = lidarPWMRight0Buffer.getMedian() == MathHelper.clamp(lidarPWMLeft0Buffer.getMedian(), lidarPWMRight0Buffer.getMedian() - lidar0AllowedVariability, lidarPWMRight0Buffer.getMedian() + lidar0AllowedVariability);
    }

    private double getFromBumperMeasurement(ReadingBuffer buffer){
//        if (buffer.getMedian() <= bumperDistance){
//            return 0.00;
//        }
//        else {
//            //This should be the distance (In meters) that the Lidar is from the edge of the front bumper
//            return buffer.getMedian() - bumperDistance;
//        }
        return buffer.getMedian() - bumperDistance;
    }

    public boolean angledLidarHitLeftClose(){
        return angledLidarLeftDistance() - getFromBumperMeasurement(lidarPWMLeft0Buffer) < minHitDistance;
    }

    public boolean angledLidarHitLeftFar(){
        return angledLidarLeftDistance() - getFromBumperMeasurement(lidarPWMLeft0Buffer) > maxHitDistance;
    }

    public boolean angledLidarHitRightClose(){
        return angledLidarRightDistance() - getFromBumperMeasurement(lidarPWMRight0Buffer) < minHitDistance;
    }

    public boolean angledLidarHitRightFar(){
        return angledLidarRightDistance() - getFromBumperMeasurement(lidarPWMRight0Buffer) > maxHitDistance;
    }

    public double angledLidarLeftDistance(){
        return getFromBumperMeasurement(lidarPWMLeft60Buffer);
    }

    public double angledLidarRightDistance(){
        return getFromBumperMeasurement(lidarPWMRight60Buffer);
    }

    public void enableLidar(){
        lidars.forEach(LidarPWM::turnOn);
    }

    public void disableLidar(){
        lidars.forEach(LidarPWM::turnOff);
        buffers.forEach(ReadingBuffer::clear);
    }

    public boolean lidarLeft0IsCloser() {
        return getFromBumperMeasurement(lidarPWMLeft0Buffer) < getFromBumperMeasurement(lidarPWMRight0Buffer);
    }

    public boolean lidarRight0IsCloser(){
        return getFromBumperMeasurement(lidarPWMLeft0Buffer) > getFromBumperMeasurement(lidarPWMRight0Buffer);
    }

    public double getDistanceFromReef(){
        double lidarDifference = getLidarDifference();
        if (lidarLeft0IsCloser()){
            return getFromBumperMeasurement(lidarPWMLeft0Buffer);
        }
        else  {
            return getFromBumperMeasurement(lidarPWMRight0Buffer);
        }
    }

    public double getLidarDifference(){
        return Math.abs(getFromBumperMeasurement(lidarPWMLeft0Buffer) - getFromBumperMeasurement(lidarPWMRight0Buffer));
    }

    public boolean lidars0DontEqual(){
        return !lidarRight0EqualToLeft0;
    }

    public boolean isTouching() {
        return !lidars0DontEqual() && getDistanceFromReef() < touchingDistance;
    }

    public double getSkew() {
        //assume 90 degrees is left and -90 is right
        return lidarLeft0IsCloser() ? getTurnAngle() * -1 : getTurnAngle();
    }

    public double getTurnAngle(){
        double lidarDifference = getLidarDifference();
        return Math.toDegrees(Math.atan2(lidarDifference, Constants.BluePositions.cRobotWidth));
    }

    private static class ReadingBuffer{
        private final int bufferSize;
        private final List<Double> buffer;
        private int counter;
        private final Supplier<Double> readings;
        private final Supplier<Boolean> enabled;

        public ReadingBuffer(int bufferSize, Supplier<Double> readings, Supplier<Boolean> enabled){
            counter = 0;
            this.bufferSize = bufferSize;
            this.readings = readings;
            this.enabled = enabled;
            this.buffer = new ArrayList<>(bufferSize);
            this.clear();
        }

        public ReadingBuffer(int bufferSize){
            this(bufferSize, () -> 0.0, () -> true);
        }

        public void add(double reading){
            if(buffer.size() - 1 < counter){
                buffer.add(reading);
            }
            else {
                buffer.set(counter, reading);
            }
            counter = (counter + 1) % bufferSize;
        }

        public void add(){
            if(enabled.get()){
                add(readings.get());
            }
        }

        public double getMedian(){
            if(buffer.isEmpty()){
                return 0.0;
            }
            else {
                int index = (int)Math.min(buffer.size()/2.0, buffer.size() - 1);
                return buffer.stream().sorted().toList().get(index);
            }
        }

        public void clear(){
            this.buffer.clear();
        }

    }


}
