package frc.robot.positioning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BlueOrientation implements Orientation{

    protected BlueOrientation() {}

    @Override
    public Pose2d getReefA() { return new Pose2d(3.16, 4.19, Rotation2d.fromDegrees(0)); }

    @Override
    public Pose2d getReefB() { return new Pose2d(3.16, 3.86, Rotation2d.fromDegrees(0)); }

    @Override
    public Pose2d getReefC() { return new Pose2d(3.68, 2.96, Rotation2d.fromDegrees(60)); }

    @Override
    public Pose2d getReefD() { return new Pose2d(3.97, 2.80, Rotation2d.fromDegrees(60)); }

    @Override
    public Pose2d getReefE() { return new Pose2d(5.00, 2.80, Rotation2d.fromDegrees(120)); }

    @Override
    public Pose2d getReefF() { return new Pose2d(5.30, 2.96, Rotation2d.fromDegrees(120)); }

    @Override
    public Pose2d getReefG() { return new Pose2d(5.82, 3.86, Rotation2d.fromDegrees(180)); }

    @Override
    public Pose2d getReefH() { return new Pose2d(5.82, 4.19, Rotation2d.fromDegrees(180)); }

    @Override
    public Pose2d getReefI() { return new Pose2d(5.30, 5.09, Rotation2d.fromDegrees(-120)); }

    @Override
    public Pose2d getReefJ() { return new Pose2d(5.00, 5.25, Rotation2d.fromDegrees(-120)); }

    @Override
    public Pose2d getReefK() { return new Pose2d(3.97, 5.25, Rotation2d.fromDegrees(-60)); }

    @Override
    public Pose2d getReefL() { return new Pose2d(3.68, 5.09, Rotation2d.fromDegrees(-60)); }

    @Override
    public Pose2d getProcessor() { return new Pose2d(11.51, 7.47, Rotation2d.fromDegrees(90)); }

    @Override
    public Pose2d getCoralStationLA() { return new Pose2d(0.76, 6.69, Rotation2d.fromDegrees(36)); }

    @Override
    public Pose2d getCoralStationLB() { return new Pose2d(1.19, 7, Rotation2d.fromDegrees(36)); }

    @Override
    public Pose2d getCoralStationLC() { return new Pose2d(1.63, 7.32, Rotation2d.fromDegrees(36)); }

    @Override
    public Pose2d getCoralStationRA() { return new Pose2d(0.76, 1.34, Rotation2d.fromDegrees(-36)); }

    @Override
    public Pose2d getCoralStationRB() { return new Pose2d(1.19, 1.02, Rotation2d.fromDegrees(-36)); }

    @Override
    public Pose2d getCoralStationRC() { return new Pose2d(1.63, 0.71, Rotation2d.fromDegrees(-36)); }

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
