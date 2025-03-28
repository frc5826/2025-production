package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.FilterLOF2D;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import static frc.robot.Constants.*;

public class CameraSystem {


    private static final double POSE_CUTOFF = 0.2;
    private static final double distanceCutoff = 4;

    private static AprilTagFieldLayout fieldLayout;
    private final List<Camera> aprilTagCameras;
    private Camera reefCamera;
    private DoubleLogEntry xLog, yLog, rotationLog, ambiguityLog;

    private FilterLOF2D filter;
    private LinkedList<Pose3d> tagLog;

    public CameraSystem() {

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
            fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            fieldLayout = null;
            System.err.println("April Tag Field Layout Failed to Load");
            e.printStackTrace();
        }

        aprilTagCameras = List.of(
                new Camera(new Translation3d(inToM(-3), inToM(8.5), inToM(12.125)),
                        new Rotation3d(0, -Math.PI / 4, 0),
                        "L45"),
                new Camera(new Translation3d(inToM(-3), inToM(8.5), 0.7),
                        new Rotation3d(0, Math.toRadians(25), 0),
                        "L0"),
                new Camera(new Translation3d(inToM(-3), inToM(-8.5), inToM(12.125)),
                        new Rotation3d(Math.PI, -Math.PI / 4, 0),
                        "R45"),
                new Camera(new Translation3d(inToM(-3), inToM(-8.5), 0.7),
                        new Rotation3d(0, Math.toRadians(25),0),
                        "R0")
        );

//        reefCamera = new Camera(new Translation3d(inToM(14), 0, inToM(4)),
//                new Rotation3d(0, Math.toRadians(80), 0),
//                "Center");

//        DataLog log = DataLogManager.getLog();
//        xLog = new DoubleLogEntry(log, "/robot/vision/position/x");
//        yLog = new DoubleLogEntry(log, "/robot/vision/position/y");
//        rotationLog = new DoubleLogEntry(log, "/robot/vision/position/rotation");
//        ambiguityLog = new DoubleLogEntry(log, "/robot/vision/ambiguity");

        filter = new FilterLOF2D(cLOFk);

        tagLog = new LinkedList<>();

    }

    public Optional<Double> getReefTargetYaw() {
        for (var result : reefCamera.getCamera().getAllUnreadResults()) {
            if (result.hasTargets()) {
                return Optional.of(result.getBestTarget().getYaw());
            }
        }
        return Optional.empty();
    }

    public List<VisionMeasurement> getCameraMeasurements() {

        LinkedList<VisionMeasurement> results = new LinkedList<>();
        LinkedList<VisionMeasurement> filteredResults = new LinkedList<>();

        for (Camera camera : aprilTagCameras) {

            for (PhotonPipelineResult result : camera.getCamera().getAllUnreadResults()) {

                if (result.hasTargets()) {

                    for (PhotonTrackedTarget target : result.getTargets()) {
                        if (target.getFiducialId() > -1 &&
                                target.getPoseAmbiguity() <= POSE_CUTOFF &&
                                target.getPoseAmbiguity() != -1 &&
                                Math.hypot(target.getBestCameraToTarget().getX(), target.getBestCameraToTarget().getY()) < distanceCutoff) { //TODO test if distance rejection works

                            Pose3d robotPose = getRobotLocation(camera.getCameraToRobot(), target.getBestCameraToTarget(), target.getFiducialId());

//                            xLog.append(robotPose.getX());
//                            yLog.append(robotPose.getY());
//                            rotationLog.append(robotPose.getRotation().getZ());
//                            ambiguityLog.append(target.getPoseAmbiguity());

//                            SmartDashboard.putNumber("Cameras/" + camera.getName() + "/x", robotPose.getX());
//                            SmartDashboard.putNumber("Cameras/" + camera.getName() + "/y", robotPose.getY());
//                            SmartDashboard.putNumber("Cameras/" + camera.getName() + "/yaw", robotPose.getRotation().getZ());
//                            SmartDashboard.putNumber("Cameras/" + camera.getName() + "/time_stamp", result.getTimestampSeconds());
//                            SmartDashboard.putNumber("Ambiguity", target.getPoseAmbiguity());

                            results.add(new VisionMeasurement(robotPose,target.getFiducialId()));
                            tagLog.add(robotPose);
                            if (tagLog.size() > cLOFTagLimit) {
                                tagLog.removeFirst();
                            }
                        }

                    }
                }

            }
        }

        for (var possible : results) {
            double factor = filter.LOF(possible.robotPose(), tagLog);
            if (factor <= cLOFRejectionValue) {
                filteredResults.add(possible);
            }
        }

        return filteredResults;

    }

    private Pose3d getRobotLocation(Transform3d cameraToRobot, Transform3d aprilTagLocation, int aprilTagID) {
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(aprilTagID);

        Pose3d robotPose = null;

        if (tagPose.isPresent()) {
            Pose3d camPose = tagPose.get().transformBy(aprilTagLocation.inverse());
            robotPose = camPose.transformBy(cameraToRobot);
        }

        return robotPose;
    }

    public static AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    private double inToM(double inch) {
        return inch * 0.0254;
    }

    public static class Camera {

        private final Transform3d cameraToRobot;
        private final PhotonCamera camera;

        public Camera(Translation3d cameraLocation, Rotation3d cameraDirection, String cameraName) {

            camera = new PhotonCamera(cameraName);
            cameraToRobot = new Transform3d(cameraLocation, cameraDirection).inverse();

        }

        public Transform3d getCameraToRobot() {
            return cameraToRobot;
        }

        public PhotonCamera getCamera() {
            return camera;
        }

        public String getName() {
            return camera.getName();
        }
    }

}

