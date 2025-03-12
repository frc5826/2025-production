package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.ReefPosition;

import java.util.List;

public class ReefTargeting extends SubsystemBase {

    private ReefPosition target;
    private Pose2d pose;
    private ReefPosition.ReefLevel level;

    private List<ReefPosition> autoList;

    public ReefTargeting() { //TODO set auto paths to the first targets
        target = new ReefPosition(new Pose2d(0, 0, new Rotation2d(0)), ReefPosition.ReefLevel.NONE);
        pose = new Pose2d(0, 0, new Rotation2d(0));
        level = ReefPosition.ReefLevel.NONE;
    }

    public void updateTarget(ReefPosition target) {
        this.target = target;
    }

    public ReefPosition getTarget() {
        return target;
    }

    public void updateLevel(ReefPosition.ReefLevel level) {
        this.level = level;
        target = new ReefPosition(this.pose, this.level);
    }

    public ReefPosition.ReefLevel getLevel() {
        return level;
    }

    public void updatePose(Pose2d pose) {
        this.pose = pose;
        target = new ReefPosition(this.pose, this.level);
        System.out.println("Updating goal pose to: " + this.pose);
    }

    public Pose2d getPose() {
        return pose;
    }

}
