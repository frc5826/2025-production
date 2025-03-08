package frc.robot.commands.swerve.pathing;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.PID;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.nio.DoubleBuffer;
import java.util.ArrayDeque;
import java.util.Optional;

public class AlignReefCameraCommand extends LoggedCommand {

    private SwerveSubsystem s;
    private CameraSubsystem ca;

    private double yaw;
    private boolean finished;
    private final double farYawThreshold = 9;
    private double averageYaw;
    private final int averagePastCount = 5;
    private ArrayDeque<Double> pastYaws;

    private PID crabwalkPID = new PID(0.05, 0.001, 0, 0.8, -0.8, 0.3, this::getYaw);

    public AlignReefCameraCommand(CameraSubsystem cameraSubsystem, SwerveSubsystem swerveSubsystem) {
        this.s = swerveSubsystem;
        this.ca = cameraSubsystem;

        yaw = 0;
        finished = false;
        averageYaw = 0;

        pastYaws = new ArrayDeque<Double>(averagePastCount);

        SmartDashboard.putData("align camera pid", crabwalkPID);

        addRequirements(s);
    }

    @Override
    public void initialize() {
        super.initialize();

        pastYaws.clear();
        finished = false;

        crabwalkPID.setGoal(0);
    }

    @Override
    public void execute() {
        super.execute();

        Optional<Double> result = ca.getReefYaw();

        if (result.isPresent()) {
            yaw = result.get();

            pastYaws.add(yaw);
            if (pastYaws.size() > averagePastCount) {
                pastYaws.removeFirst();
            }

            averageYaw = 0;
            for (Double i : pastYaws) {
                averageYaw += i;
            }
            averageYaw /= averagePastCount;

            System.out.println("average yaw: " + averageYaw);

            if (Math.abs(averageYaw) > farYawThreshold) {
                finished = true;
            }

        }

        crabwalkPID.calculate();
        s.driveRobotOriented(new ChassisSpeeds(0, crabwalkPID.getOutput(), 0));
    }

    private double getYaw() {
        return yaw;
    }

    @Override
    public boolean isFinished() {
        return finished || crabwalkPID.getDeadband() > Math.abs(yaw);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
