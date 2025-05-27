package frc.robot.positioning;

import edu.wpi.first.math.geometry.Pose2d;
import static frc.robot.Constants.Elevator.*;

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
        NONE(0, 30),
        L1(L1Height, L1Angle),
        L2(L2Height, L2Angle),
        L3(L3Height, L3Angle),
        L4(L4Height, L4Angle),
        ALGL2(AlgL2Height, AlgL2Angle),
        ALGL3(AlgL3Height, AlgL3Angle);

        public final double height;
        public final double angle;

        private ReefLevel(double height, double angle) {
            this.height = height;
            this.angle = angle;
        }
    }

}
