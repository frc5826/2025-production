package frc.robot.localization;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private Field2d field;

    public Localization(CameraSubsystem cameraSubsystem) {
        kalmanFilter = new KalmanFilter(new MultivariateNormalDistribution(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0}, initCovar()));
        this.measVar = new Variances(5, 1.2, 8, 4, 1.2, 8);
        this.cameraSubsystem = cameraSubsystem;
        timer = new Timer();

        field = new Field2d();
        setupFieldTab();
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

            Rotation2d angleDiff = p.getRotation().toRotation2d().minus(s.getIMUContinuousAngle());

            zOdo.setEntry(2, s.getIMUContinuousAngle().getRadians()+angleDiff.getRadians());

            ROdo.setEntry(2, 2, measVar.rPos());
            SmartDashboard.putNumber("cam Diff", angleDiff.getDegrees());
            SmartDashboard.putNumber("continuous gyro angle", s.getIMUContinuousAngle().getDegrees());

            kalmanFilter.measure(ROdo, zOdo);
        }

        zOdo = kalmanFilter.getX().copy();
        ROdo = kalmanFilter.getP().copy();

        //Acceleration from accelerometer
        Optional<Translation3d> accelOptional = s.getAcc();
        if(accelOptional.isPresent()){
            zOdo.setEntry(6, accelOptional.get().getX());
            zOdo.setEntry(7, accelOptional.get().getY());

            ROdo.setEntry(6, 6, measVar.xyAcc());
            ROdo.setEntry(7, 7, measVar.xyAcc());
        }

        //Velocity from wheel encoders
        ChassisSpeeds velocity = s.getOdoVel();
        zOdo.setEntry(3, velocity.vxMetersPerSecond);
        zOdo.setEntry(4, velocity.vyMetersPerSecond);
        zOdo.setEntry(5, velocity.omegaRadiansPerSecond);

        ROdo.setEntry(3, 3, measVar.xyVel());
        ROdo.setEntry(4, 4, measVar.xyVel());
        ROdo.setEntry(5, 5, measVar.rVel());

        //Rotation from gyro
        zOdo.setEntry(2, s.getIMUContinuousAngle().getRadians());

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
        SmartDashboard.putNumber("step 1", Math.toDegrees(m.getEntry(2)));
        SmartDashboard.putNumber("step 2", Math.toDegrees(Math.abs(m.getEntry(2)) % (Math.PI * 2)));
        if (yaw > Math.PI) {yaw -= Math.PI * 2;}
        SmartDashboard.putNumber("step 3", Math.toDegrees(yaw));

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

    public void updateField() {
        field.setRobotPose(getPose());
    }

    private void setupFieldTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("field");

        field = new Field2d();
        tab.add(field)
                .withPosition(2,0)
                .withSize(5,3);

        ShuffleboardLayout position = tab.getLayout("Robot position", BuiltInLayouts.kList)
                .withPosition(0,0)
                .withSize(2,2);

        position.addDouble("Robot X", ()-> getPose().getX());
        position.addDouble("Robot Y", ()-> getPose().getY());
        position.addDouble("Robot rotation", ()-> getPose().getRotation().getDegrees());
    }
}
