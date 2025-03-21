package frc.robot.positioning;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import static frc.robot.Constants.BluePositions.*;

public class RedOrientation implements Orientation {

    protected RedOrientation() {}

    @Override
    public Pose2d getReefA() { return FlippingUtil.flipFieldPose(reefA); }

    @Override
    public Pose2d getReefB() { return FlippingUtil.flipFieldPose(reefB); }

    @Override
    public Pose2d getReefC() { return FlippingUtil.flipFieldPose(reefC); }

    @Override
    public Pose2d getReefD() { return FlippingUtil.flipFieldPose(reefD); }

    @Override
    public Pose2d getReefE() {
        return FlippingUtil.flipFieldPose(reefE);
    }

    @Override
    public Pose2d getReefF() { return FlippingUtil.flipFieldPose(reefF); }

    @Override
    public Pose2d getReefG() { return FlippingUtil.flipFieldPose(reefG); }

    @Override
    public Pose2d getReefH() { return FlippingUtil.flipFieldPose(reefH); }

    @Override
    public Pose2d getReefI() { return FlippingUtil.flipFieldPose(reefI); }

    @Override
    public Pose2d getReefJ() { return FlippingUtil.flipFieldPose(reefJ); }

    @Override
    public Pose2d getReefK() { return FlippingUtil.flipFieldPose(reefK); }

    @Override
    public Pose2d getReefL() { return FlippingUtil.flipFieldPose(reefL); }

    @Override
    public Pose2d getReefSideAB() { return FlippingUtil.flipFieldPose(reefSideAB); }

    @Override
    public Pose2d getReefSideCD() { return FlippingUtil.flipFieldPose(reefSideCD); }

    @Override
    public Pose2d getReefSideEF() { return FlippingUtil.flipFieldPose(reefSideEF); }

    @Override
    public Pose2d getReefSideGH() { return FlippingUtil.flipFieldPose(reefSideGH); }

    @Override
    public Pose2d getReefSideIJ() { return FlippingUtil.flipFieldPose(reefSideIJ); }

    @Override
    public Pose2d getReefSideKL() { return FlippingUtil.flipFieldPose(reefSideKL); }

    @Override
    public Pose2d getProcessor() { return FlippingUtil.flipFieldPose(processor); }

    @Override
    public Pose2d getCoralStationLA() { return FlippingUtil.flipFieldPose(coralStationLA); }

    @Override
    public Pose2d getCoralStationLB() { return FlippingUtil.flipFieldPose(coralStationLB); }

    @Override
    public Pose2d getCoralStationLC() { return FlippingUtil.flipFieldPose(coralStationLC); }

    @Override
    public Pose2d getCoralStationRA() { return FlippingUtil.flipFieldPose(coralStationRA); }

    @Override
    public Pose2d getCoralStationRB() { return FlippingUtil.flipFieldPose(coralStationRB); }

    @Override
    public Pose2d getCoralStationRC() { return FlippingUtil.flipFieldPose(coralStationRC); }

    @Override
    public double getStartOrientation() {
        return 0;
    }

    @Override
    public double getDriveOrientation() {
        return 180;
    }

    @Override
    public boolean isValid() {
        return true;
    }

}
