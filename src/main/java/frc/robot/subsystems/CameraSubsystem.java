package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.sensors.CameraSystem;
import frc.robot.sensors.VisionMeasurement;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CameraSubsystem extends LoggedSubsystem {

    private CameraSystem cameraSystem;
    private List<VisionMeasurement> measurements;

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

    public List<VisionMeasurement> getCameraMeasurements() {
        return measurements;
    }

    public Optional<Double> getReefYaw() {
        return cameraSystem.getReefTargetYaw();
    }

    public boolean hasReefTarget() {
        return cameraSystem.getReefTargetYaw().isPresent();
    }
}
