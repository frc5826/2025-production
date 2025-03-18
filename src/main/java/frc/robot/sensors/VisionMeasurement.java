package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose3d;

public record VisionMeasurement(Pose3d robotPose, int tagID) {}
