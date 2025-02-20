package frc.robot.math;

import edu.wpi.first.math.geometry.*;

public class MathHelper {

    public static Rotation2d getAngleAtoB(Pose2d A, Pose2d B) {
        return B.getTranslation().minus(A.getTranslation()).getAngle();
    }

    public static Pose2d offsetPoseReverse(Pose2d pose, double offset) {
        return new Pose2d(pose.getX() + (offset * Math.cos(pose.getRotation().getRadians())),
                pose.getY() + (offset * Math.sin(pose.getRotation().getRadians())),
                pose.getRotation());
    }

    //TODO Yoink wut
    public static double clamp(double value, double min, double max) {
        // This unusual condition allows keeping only one branch
        // on common path when min < max and neither of them is NaN.
        // If min == max, we should additionally check for +0.0/-0.0 case,
        // so we're still visiting the if statement.
        if (!(min < max)) { // min greater than, equal to, or unordered with respect to max; NaN values are unordered
            if (Double.isNaN(min)) {
                throw new IllegalArgumentException("min is NaN");
            }
            if (Double.isNaN(max)) {
                throw new IllegalArgumentException("max is NaN");
            }
            if (Double.compare(min, max) > 0) {
                throw new IllegalArgumentException(min + " > " + max);
            }
            // Fall-through if min and max are exactly equal (or min = -0.0 and max = +0.0)
            // and none of them is NaN
        }
        return Math.min(max, Math.max(value, min));
    }

}
