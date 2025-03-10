package frc.robot.positioning;

import edu.wpi.first.math.geometry.Pose2d;

public interface Orientation {

    Pose2d getReefA();

    Pose2d getReefB();

    Pose2d getReefC();

    Pose2d getReefD();

    Pose2d getReefE();

    Pose2d getReefF();

    Pose2d getReefG();

    Pose2d getReefH();

    Pose2d getReefI();

    Pose2d getReefJ();

    Pose2d getReefK();

    Pose2d getReefL();

    Pose2d getReefSideAB();

    Pose2d getReefSideCD();

    Pose2d getReefSideEF();

    Pose2d getReefSideGH();

    Pose2d getReefSideIJ();

    Pose2d getReefSideKL();

    Pose2d getProcessor();

    Pose2d getCoralStationLA();

    Pose2d getCoralStationLB();

    Pose2d getCoralStationLC();

    Pose2d getCoralStationRA();

    Pose2d getCoralStationRB();

    Pose2d getCoralStationRC();

    double getStartOrientation();

    double getDriveOrientation();

    boolean isValid();

}
