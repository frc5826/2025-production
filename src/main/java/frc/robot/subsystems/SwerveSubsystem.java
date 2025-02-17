package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.localization.Localization;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
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

    private double speedMultiplier;

    private AHRS navX;//TODO replace all instances of (AHRS)swerveDrive.getGyro().getIMU()

    private double imuOffset = 0;
    private Orientation orientation = FieldOrientation.unknownOrientation;

    private double driveAngleOffset;

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

        speedMultiplier = cLowSpeedMultiplier;

        navX = (AHRS)swerveDrive.getGyro().getIMU();

        setupPathPlanner();
    }

    public Pose2d getLocalizationPose() { return localization.getPose(); }

    //old get accel
//    public Optional<Translation3d> getAcc() {
//        if(swerveDrive.getAccel().isPresent()) {
//            var t = swerveDrive.getAccel().get()
//                    .rotateBy(((AHRS)swerveDrive.getGyro().getIMU()).getRotation3d().unaryMinus())
//                    .rotateBy(swerveDrive.getGyroRotation3d().unaryMinus());
//            return Optional.of(new Translation3d(t.getX(),t.getY(),t.getZ()));
//        } else
//            return Optional.empty();
//    }

    public Optional<Translation3d> getFieldAcc() {
        if (swerveDrive.getAccel().isPresent()) {
            Translation3d acc = swerveDrive.getAccel().get();
            //TODO - Ryan's Notes - When you call rotateBy(), does it change the object you are passing in? If not, how do you get access to the rotated Translation?
            acc.rotateBy(navX.getRotation3d().unaryMinus());
            //TODO - Ryan's Notes - This doesn't like being a minus.
            acc.rotateBy(new Rotation3d(getAdjustedIMUContinuousAngle().unaryMinus()));
            return Optional.of(acc);
        } else {
            return Optional.empty();
        }
    }

    public Orientation getOrientation() {
        return orientation;
    }

    public void setOrientation(Orientation orientation) {
        this.orientation = orientation;
        imuOffset = orientation.getStartOrientation() - getIMUContinuousAngle().getDegrees();
    }

    public void setSpeedMultiplier(boolean fast) {
        speedMultiplier = fast ? cHighSpeedMultiplier : cLowSpeedMultiplier;
    }

    public double getSpeedMultiplier() { return speedMultiplier; }

    //old get vel
    public ChassisSpeeds getOdoVel() { return swerveDrive.getFieldVelocity(); }

    public ChassisSpeeds getOdoFieldVel() {
        ChassisSpeeds vel = swerveDrive.getRobotVelocity();
        Translation2d velTranslation = new Translation2d(vel.vxMetersPerSecond, vel.vyMetersPerSecond);
        //TODO - Ryan's Notes - When you call rotateBy(), does it change the object you are passing in? If not, how do you get access to the rotated Translation?
        //TODO - Ryan's Notes - This doesn't like being a minus.
        velTranslation.rotateBy(getAdjustedIMUContinuousAngle().unaryMinus());
        vel = new ChassisSpeeds(velTranslation.getX(), velTranslation.getY(), vel.omegaRadiansPerSecond);
        return vel;
    }

    //broken
    public void driveFieldOriented(ChassisSpeeds velocity) {
        //TODO getYaw gets the value from swervedrive.getOdometry, which has an internal offset
        //  which is doubled up with our offset
        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(velocity,
                navX.getRotation2d().plus(new Rotation2d(-driveAngleOffset))));
    }

    public void teleDriveFieldOriented(ChassisSpeeds vel) {
        //TODO - Ryan's Notes - What unit is getAdjustedIMUContinuousAngle and orientation.getDriveOrientation() in? What unit is fromFieldRelativeSpeeds() expecting?
        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vel, getAdjustedIMUContinuousAngle().minus(new Rotation2d(orientation.getDriveOrientation())))); //TODO xbox y might not need a negative
    }

    public void driveSRSFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public void resetOdometry(Pose2d pose) { swerveDrive.resetOdometry(pose); }

    public Rotation2d getIMUContinuousAngle() {
        //TODO - Ryan's Notes - Positive rotation on the NavX is CW, but on the field it's CCW.
        return Rotation2d.fromDegrees(navX.getAngle());
    }

    public Rotation2d getAdjustedIMUContinuousAngle() {
        //TODO - Ryan's Notes - Positive rotation on the NavX is CW, but on the field it's CCW.
        return Rotation2d.fromDegrees(navX.getAngle() + imuOffset);
    }

    public Rotation2d getYaw() {
        return swerveDrive.getYaw();
    }

    public void resetDriveGyro() {
        driveAngleOffset = swerveDrive.getGyro().getRawRotation3d().getZ();
    }

    public void setDriveGyroOffset(double offsetRad) {
        driveAngleOffset = swerveDrive.getGyro().getRawRotation3d().getZ() + offsetRad;
    }

    public void zeroOdoGyro(double offset) {
        swerveDrive.setGyro(new Rotation3d(0, 0, offset));
    }

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
