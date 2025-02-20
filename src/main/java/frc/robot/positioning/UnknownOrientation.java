package frc.robot.positioning;

import edu.wpi.first.math.geometry.Pose2d;

public class UnknownOrientation implements Orientation {

    protected UnknownOrientation() {}

    @Override
    public Pose2d getReefA() { return null; };

    @Override
    public Pose2d getReefB() { return null; }

    @Override
    public Pose2d getReefC() { return null; }

    @Override
    public Pose2d getReefD() { return null; }

    @Override
    public Pose2d getReefE() { return null; }

    @Override
    public Pose2d getReefF() { return null; }

    @Override
    public Pose2d getReefG() { return null; }

    @Override
    public Pose2d getReefH() { return null; }

    @Override
    public Pose2d getReefI() { return null; }

    @Override
    public Pose2d getReefJ() { return null; }

    @Override
    public Pose2d getReefK() { return null; }

    @Override
    public Pose2d getReefL() { return null; }

    @Override
    public Pose2d getProcessor() { return null; }

    @Override
    public Pose2d getCoralStationLA() { return null; }

    @Override
    public Pose2d getCoralStationLB() { return null; }

    @Override
    public Pose2d getCoralStationLC() { return null; }

    @Override
    public Pose2d getCoralStationRA() { return null; }

    @Override
    public Pose2d getCoralStationRB() { return null; }

    @Override
    public Pose2d getCoralStationRC() { return null; }

    @Override
    public double getStartOrientation() {
        return 0;
    }

    @Override
    public double getDriveOrientation() {
        return 0;
    }

        @Override
    public boolean isValid() {
        return false;
    }
}
