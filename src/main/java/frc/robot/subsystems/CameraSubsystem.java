package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.sensors.CameraSystem;

import java.util.ArrayList;
import java.util.List;

public class CameraSubsystem extends LoggedSubsystem {

    private CameraSystem cameraSystem;
    private List<Pose3d> measurements;

    private Pose2d aligningPose;

    public CameraSubsystem() {
        cameraSystem = new CameraSystem();
        measurements = new ArrayList<>();
    }

    @Override
    public void periodic() {
        super.periodic();

        measurements = cameraSystem.getCameraMeasurements();
    }

    public List<Pose3d> getCameraMeasurements() {
        return measurements;
    }
}
