package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.sensors.CameraSystem;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CameraSubsystem extends LoggedSubsystem {

    private CameraSystem cameraSystem;
    private List<CameraSystem.Result> measurements;

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
        return measurements.stream().map(CameraSystem.Result::robotPose).toList();
    }

    public Optional<Double> getReefYaw() {
        return cameraSystem.getReefTargetYaw();
    }

    public boolean hasReefTarget() {
        return cameraSystem.getReefTargetYaw().isPresent();
    }

    public Optional<Pose3d> getRobotPose(int id, boolean left){
        for (CameraSystem.Result result : measurements){
            if (result.id() == id && left && result.camera().equals(cameraSystem.getLeftForward())){
                return Optional.of(result.robotPose());
            }
            else if (result.id() == id && !left && result.camera().equals(cameraSystem.getRightForward())) {
                return Optional.of(result.robotPose());
            }
        }
        return Optional.empty();
    }

}
