package frc.robot.positioning;

import edu.wpi.first.math.geometry.Pose2d;

public class ReefPosition {

    private Pose2d position;
    private ReefLevel level;

    public ReefPosition(Pose2d position, ReefLevel level) {
        this.position = position;
        this.level = level;
    }

    public Pose2d getPosition() {
        return position;
    }

    public ReefLevel getLevel() {
        return level;
    }

    public enum ReefLevel {
        NONE,
        L1,
        L2,
        L3,
        L4
    }

}
