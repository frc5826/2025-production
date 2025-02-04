package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.localization.Localization;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.util.Optional;

import static frc.robot.Constants.Swerve.*;

public class SwerveSubsystem extends LoggedSubsystem {

    private final SwerveDrive swerveDrive;

    private double maximumSpeed = cMaxVelocity;

    private Rotation2d targetAngle = new Rotation2d();

    private Localization localization;

    public SwerveSubsystem(Localization localization) {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
                12.8, 1);

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(4), 6.12, 1);

        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory() + "/swerve")).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        SmartDashboard.putData("/drive/ahrs",(AHRS)swerveDrive.getGyro().getIMU());

        resetOdometry(new Pose2d(0, 0, new Rotation2d()));

        this.localization = localization;

    }

    public void zeroGyro()
    {
        targetAngle = new Rotation2d();
        swerveDrive.zeroGyro();
    }

    public Pose2d getLocalizationPose() { return localization.getPose(); }

    public Optional<Translation3d> getAcc() {
        if(swerveDrive.getAccel().isPresent()) {
            var t = swerveDrive.getAccel().get()
                    .rotateBy(((AHRS)swerveDrive.getGyro().getIMU()).getRotation3d().unaryMinus())
                    .rotateBy(swerveDrive.getGyroRotation3d().unaryMinus());
            return Optional.of(new Translation3d(t.getX(),t.getY(),t.getZ()));
        } else
            return Optional.empty();
    }

    public Rotation2d getRotationCorrected() {
        return getIMUYaw().getRadians() < 0 ? getIMUYaw().plus(new Rotation2d(2 * Math.PI)) : getIMUYaw();
    }

    public ChassisSpeeds getOdoVel() { return swerveDrive.getFieldVelocity(); }

    public void driveFieldOriented(ChassisSpeeds velocity) { swerveDrive.driveFieldOriented(velocity); }

    public void resetOdometry(Pose2d pose) { swerveDrive.resetOdometry(pose); }

    public Rotation2d getIMUYaw() { return swerveDrive.getYaw(); }

    public Rotation2d getTargetAngle() { return targetAngle; }

    public void setTargetAngle(Rotation2d targetAngle) { this.targetAngle = targetAngle; }

}
