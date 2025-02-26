package frc.robot.commands.swerve.pathing;

import com.fasterxml.jackson.databind.ser.impl.ReadOnlyClassToSerializerMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.PID;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class CrabWalkDistanceCommand extends LoggedCommand {

    private SwerveSubsystem swerveSubsystem;

    private boolean finished;

    private double distanceLeft;
    private DoubleSupplier distanceSupplier;

    private Timer timer;
    private double lastTime;
    private double timeOut;

    private PID crabWalkPID;
    private PID pidR;
    private Rotation2d turnGoal;

    public CrabWalkDistanceCommand(DoubleSupplier distance, double timeOut, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        this.timeOut = timeOut;

        this.distanceSupplier = distance;

        timer = new Timer();
        crabWalkPID = new PID(2, 0.01, 0.01, 0.8, 0.5, 0.01, this::getDifference);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        distanceLeft = distanceSupplier.getAsDouble();

        timer.restart();
        finished = false;

        turnGoal = swerveSubsystem.getLocalizationPose().getRotation();
        pidR  = new PID(Constants.Swerve.cTurnPID, Math.PI, 0.01, 0.05, () -> turnGoal.minus(swerveSubsystem.getLocalizationPose().getRotation()).getRadians());

        crabWalkPID.setGoal(0);
        pidR.setGoal(0);
    }

    @Override
    public void execute() {
        super.execute();

        ChassisSpeeds speeds = new ChassisSpeeds(0, crabWalkPID.calculate(), pidR.calculate());
        swerveSubsystem.driveRobotOriented(speeds);

        if (timer.get() >= timeOut || crabWalkPID.getDeadband() >= Math.abs(getDifference())) {
            finished = true;
        }
    }

    private double getDifference() {
        distanceLeft += swerveSubsystem.getOdoRobotVel().vyMetersPerSecond * (timer.get() - lastTime);
        lastTime = timer.get();
        return distanceLeft;
    }

    @Override
    public boolean isFinished() {
        return finished || super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        timer.reset();
        finished = false;
    }
}
