package frc.robot.positioning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

public class FieldOrientation {

    public static final Orientation blueOrientation = new BlueOrientation();
    public static final Orientation redOrientation = new RedOrientation();
    public static final Orientation unknownOrientation = new UnknownOrientation();

    //We should never be using this for position. But on the off-chance this gets called, make it the middle-ish of the field.
    protected static final Pose2d NOTHING_POSE = new Pose2d(17.55/2, 7.5, Rotation2d.fromDegrees(0));

    public static Orientation getOrientation(){
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){
            System.err.println("WARNING - NO ALLIANCE SET FROM DRIVER STATION - SETTING TO UNKNOWN");
            return unknownOrientation;
        } else if (alliance.get().equals(DriverStation.Alliance.Red)) {
            return redOrientation;
        } else if(alliance.get().equals(DriverStation.Alliance.Blue)){
            return blueOrientation;
        } else {
            System.err.println("WARNING - UNKNOWN ALLIANCE FROM DRIVER STATION (" + alliance.get() + ") - SETTING TO UNKNOWN");
            return unknownOrientation;
        }
    }

}
