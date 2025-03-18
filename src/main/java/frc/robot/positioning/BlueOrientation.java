package frc.robot.positioning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.BluePositions.*;

import static frc.robot.Constants.BluePositions.*;

public class BlueOrientation implements Orientation{

    protected BlueOrientation() {}

    @Override
    public Pose2d getReefA() { return reefA; }

    @Override
    public Pose2d getReefB() { return reefB; }

    @Override
    public Pose2d getReefC() { return reefC; }

    @Override
    public Pose2d getReefD() { return reefD; }

    @Override
    public Pose2d getReefE() {
        System.out.println("Getting reefE BLUE!!!!!!!");
        return reefE;
    }

    @Override
    public Pose2d getReefF() { return reefF; }

    @Override
    public Pose2d getReefG() { return reefG; }

    @Override
    public Pose2d getReefH() { return reefH; }

    @Override
    public Pose2d getReefI() { return reefI; }

    @Override
    public Pose2d getReefJ() { return reefJ; }

    @Override
    public Pose2d getReefK() { return reefK; }

    @Override
    public Pose2d getReefL() { return reefL; }

    @Override
    public Pose2d getReefSideAB() { return reefSideAB; }

    @Override
    public Pose2d getReefSideCD() { return reefSideCD; }

    @Override
    public Pose2d getReefSideEF() { return reefSideEF; }

    @Override
    public Pose2d getReefSideGH() { return reefSideGH; }

    @Override
    public Pose2d getReefSideIJ() { return reefSideIJ; }

    @Override
    public Pose2d getReefSideKL() { return reefSideKL; }

    @Override
    public Pose2d getProcessor() { return processor; }

    @Override
    public Pose2d getCoralStationLA() { return coralStationLA; }

    @Override
    public Pose2d getCoralStationLB() { return coralStationLB; }

    @Override
    public Pose2d getCoralStationLC() { return coralStationLC; }

    @Override
    public Pose2d getCoralStationRA() { return coralStationRA; }

    @Override
    public Pose2d getCoralStationRB() { return coralStationRB; }

    @Override
    public Pose2d getCoralStationRC() { return coralStationRC; }

    @Override
    public double getStartOrientation() {
        return 180;
    }

    @Override
    public double getDriveOrientation() {
        return 0;
    }

        @Override
    public boolean isValid() {
        return true;
    }

}
