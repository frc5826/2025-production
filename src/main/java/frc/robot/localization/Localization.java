package frc.robot.localization;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.Optional;

public class Localization {

    private KalmanFilter kalmanFilter;
    private final Variances measVar;
    private final Timer timer;
    private final CameraSubsystem cameraSubsystem;
    private Pose2d cameraPose;

    public Localization(CameraSubsystem cameraSubsystem) {
        kalmanFilter = new KalmanFilter(new MultivariateNormalDistribution(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0}, initCovar()));
        this.measVar = new Variances(30, 2, 10, 4, 2, 10);
        this.cameraSubsystem = cameraSubsystem;
        timer = new Timer();
    }

    public void move() {
        kalmanFilter.move(timer.get());
        timer.restart();
    }

    public void measure(SwerveSubsystem s) {
        RealVector zOdo = kalmanFilter.getX().copy();
        RealMatrix ROdo = kalmanFilter.getP().copy();

        //Position from cameras
        for(Pose3d p: cameraSubsystem.getCameraMeasurements()) {
            cameraPose = p.toPose2d();

            zOdo.setEntry(0, p.getX());
            zOdo.setEntry(1, p.getY());

            ROdo.setEntry(0, 0, measVar.xyPos());
            ROdo.setEntry(1, 1, measVar.xyPos());

            Rotation2d angleDiff = p.getRotation().toRotation2d().minus(s.getAdjustedIMUContinuousAngle());

            zOdo.setEntry(2, s.getAdjustedIMUContinuousAngle().getRadians()+angleDiff.getRadians());

            ROdo.setEntry(2, 2, 20); //camera heading variance

            kalmanFilter.measure(ROdo, zOdo);
        }

        zOdo = kalmanFilter.getX().copy();
        ROdo = kalmanFilter.getP().copy();

        //Acceleration from accelerometer
        Optional<Translation3d> accelOptional = s.getFieldAcc();
        if(accelOptional.isPresent()){
            zOdo.setEntry(6, accelOptional.get().getX());
            zOdo.setEntry(7, accelOptional.get().getY());

            ROdo.setEntry(6, 6, measVar.xyAcc());
            ROdo.setEntry(7, 7, measVar.xyAcc());
        }

        //Velocity from wheel encoders
        ChassisSpeeds velocity = s.getOdoFieldVel();
        zOdo.setEntry(3, velocity.vxMetersPerSecond); //TODO test this and acc
        zOdo.setEntry(4, velocity.vyMetersPerSecond);
        zOdo.setEntry(5, velocity.omegaRadiansPerSecond);

        ROdo.setEntry(3, 3, measVar.xyVel());
        ROdo.setEntry(4, 4, measVar.xyVel());
        ROdo.setEntry(5, 5, measVar.rVel());

        //Rotation from gyro
        zOdo.setEntry(2, s.getAdjustedIMUContinuousAngle().getRadians());

        ROdo.setEntry(2, 2, measVar.rPos());

        //for all your readings...

        // copy z and r like above
        // enter pos in z
        // variance (r) comes from confidence of the april tag

        kalmanFilter.measure(ROdo, zOdo);
    }

    public Pose2d getPose() {
        RealVector m = kalmanFilter.getX();

        double yaw = m.getEntry(2) % (Math.PI * 2);
        if (yaw > Math.PI) {yaw -= Math.PI * 2;}

        Pose2d pose = new Pose2d(
                m.getEntry(0),
                m.getEntry(1),
                new Rotation2d(MathUtil.angleModulus(m.getEntry(2))));

        return pose;
    }

    public Pose2d getCameraPose() {
        return cameraPose;
    }

    private static double[][] initCovar() {
        return new double[][]{
                {0.1, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0.1, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0.1, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0.1, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0.1, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0.1, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0.1, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0.1, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0.1}};
    }

}

