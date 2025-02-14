package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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

    private double gyroOffset;

    private double speedMultiplier;

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
        zeroGyro();

        this.localization = localization;

        speedMultiplier = cLowSpeedMultiplier;

        setupPathPlanner();
    }

    public void zeroGyro()
    {
        targetAngle = new Rotation2d();
        swerveDrive.zeroGyro();
        gyroOffset = ((AHRS)swerveDrive.getGyro().getIMU()).getAngle() % 360;
    }

    public void setGyroOffset(double yawRads) {
        swerveDrive.setGyro(new Rotation3d(0, 0, yawRads + swerveDrive.getGyro().getRawRotation3d().getZ()));
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

    public void setSpeedMultiplier(boolean fast) {
        speedMultiplier = fast ? cHighSpeedMultiplier : cLowSpeedMultiplier;
    }

    public double getSpeedMultiplier() { return speedMultiplier; }

    public Rotation2d getRotationCorrected() {
        return getIMUYaw().getRadians() < 0 ? getIMUYaw().plus(new Rotation2d(2 * Math.PI)) : getIMUYaw();
    }

    public ChassisSpeeds getOdoVel() { return swerveDrive.getFieldVelocity(); }

    public void driveFieldOriented(ChassisSpeeds velocity) { swerveDrive.driveFieldOriented(velocity); }

    public void resetOdometry(Pose2d pose) { swerveDrive.resetOdometry(pose); }

    public Rotation2d getIMUYaw() { return swerveDrive.getYaw(); }

    public Rotation2d getIMUContinuousAngle() {
        return Rotation2d.fromDegrees(-((AHRS)swerveDrive.getGyro().getIMU()).getAngle() + gyroOffset); }

    public Rotation2d getTargetAngle() { return targetAngle; }

    public void setTargetAngle(Rotation2d targetAngle) { this.targetAngle = targetAngle; }

    public void driveRobotOriented(ChassisSpeeds velocity)
    {
        swerveDrive.drive(velocity);
    }

    public void setupPathPlanner()
    {

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            config = null;
            e.printStackTrace();
        }

        System.out.println(localization.getPose());

        // Configure AutoBuilder last
        AutoBuilder.configure(
                localization::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotOriented(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        cDrivePID, // Translation PID constants
                        cTurnPID // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }
}
