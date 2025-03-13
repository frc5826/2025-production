package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.MathHelper;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.ReefPosition;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.BluePositions.cRobotLength;

public class ReefTargeting extends SubsystemBase {

    private ReefPosition target;
    private Pose2d pose;
    private ReefPosition.ReefLevel level;

    private List<ReefPosition> autoList;

    private SwerveSubsystem s;

    public ReefTargeting(SwerveSubsystem swerveSubsystem) { //TODO set auto paths to the first targets
        target = new ReefPosition(new Pose2d(0, 0, new Rotation2d(0)), ReefPosition.ReefLevel.NONE);
        pose = new Pose2d(0, 0, new Rotation2d(0));
        level = ReefPosition.ReefLevel.NONE;

        this.s = swerveSubsystem;
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

    public Supplier<ReefPosition.ReefLevel> getLevel() {
        return () -> level;
    }

    public void updatePose(Pose2d pose) {
        this.pose = pose;
        target = new ReefPosition(this.pose, this.level);
        System.out.println("Updating goal pose to: " + this.pose);
    }

    public Pose2d getPose() {
        return pose;
    }

    public Supplier<Pose2d> getAlignmentPose() {
        return () -> MathHelper.offsetPoseReverse(pose, cRobotLength / 2);
    }

    public Supplier<Pose2d> getAlignmentOffsetPose() {
        return () -> MathHelper.offsetPoseReverse(pose, 0.35 + (cRobotLength / 2));
    }

    public Supplier<Pose2d> getFindOffsetPose() {
        return () -> MathHelper.offsetPoseReverse(pose, 1 + (cRobotLength / 2));
    }

    public BooleanSupplier isFarEnoughToPath() {
        return () -> s.getLocalizationPose().getTranslation().getDistance(getAlignmentOffsetPose().get().getTranslation()) > 1;
    }

}
