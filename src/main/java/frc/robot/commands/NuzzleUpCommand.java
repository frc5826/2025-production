package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.math.MathHelper;
import frc.robot.math.PID;
import frc.robot.positioning.AprilTag;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DistanceSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.*;

import java.util.function.BooleanSupplier;

public class NuzzleUpCommand extends LoggedCommand{

    private final DistanceSubsystem distanceSubsystem;
    private AprilTag target;
    //If true, target left, if false, target right.
    private BooleanSupplier left;
    private final CameraSubsystem cameraSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final PID nuzzlePID = new PID(10, 0, 0, Nuzzle.cZVelo, 0, Nuzzle.cZVeloDeadband, this::lidarRotational);

    public NuzzleUpCommand(DistanceSubsystem distanceSubsystem, SwerveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem, AprilTag target, BooleanSupplier left){
        this.swerveSubsystem = swerveSubsystem;
        this.distanceSubsystem = distanceSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        this.target = target;
        this.left = left;
        addRequirements(distanceSubsystem, swerveSubsystem, cameraSubsystem);
    }


    private boolean robotTooFar(){
        return left.getAsBoolean() ? distanceSubsystem.angledLidarHitLeftFar() : distanceSubsystem.angledLidarHitRightFar();
    }

    private boolean robotTooClose(){
        return left.getAsBoolean() ? distanceSubsystem.angledLidarHitLeftClose() : distanceSubsystem.angledLidarHitRightClose();
    }

    private boolean robotShouldMoveLatterly(){
        return robotTooFar() || robotTooClose();
    }

    private boolean shouldMoveForward(){
        return lidarForward();
    }

    private boolean lidarForward(){
        return !distanceSubsystem.isTouching();
    }
    //return between -1 to 1
    private double rotationalDirection(){
       if (Math.abs(lidarRotational()) > Nuzzle.cZVeloDeadband){
           return Math.signum(lidarRotational());
       }
       else {
           return 0.0;
       }
    }

    private double lidarRotational(){
        //assume 90 degrees is left and -90 is right
        double skew = distanceSubsystem.getSkew();
        double normalized = MathHelper.clamp(skew/90, -1, 1);
        return normalized * -1;
    }

    @Override
    public void initialize() {
        super.initialize();
        distanceSubsystem.enableLidar();
        nuzzlePID.setGoal(0);
    }

    @Override
    public void execute() {
        super.execute();



        double xVelo = 0.0;
        double yVelo = 0.0;
        double zVelo = 0.0; //-nuzzlePID.calculate();

        if (robotShouldMoveLatterly() && distanceSubsystem.getDistanceFromReef() < 0.4){
            if (robotTooClose()){
                xVelo += (Nuzzle.cXVelo * (left.getAsBoolean() ? 1 : -1));
            } else if (robotTooFar()) {
                xVelo -= (Nuzzle.cXVelo * (left.getAsBoolean() ? 1 : -1));
            }
        }
        if (shouldMoveForward() && !(distanceSubsystem.getDistanceFromReef() < 0.1 && robotShouldMoveLatterly())){
            yVelo += Nuzzle.cYVelo;
        }

        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(yVelo, xVelo, zVelo));
    }

    @Override
    public boolean isFinished() {
        return !shouldMoveForward() && !robotShouldMoveLatterly();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        distanceSubsystem.disableLidar();
    }
}
