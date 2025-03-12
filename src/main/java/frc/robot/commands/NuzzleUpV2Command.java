package frc.robot.commands;

import frc.robot.math.PID;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DistanceSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class NuzzleUpV2Command extends LoggedCommand{

    private DistanceSubsystem distanceSubsystem;
    private double leftRightVelocity; //I call it leftRight because it isn't always X or Y axis
    private double forwardVelocity;//I call it forward because it isn't always X or Y axis
    private CameraSubsystem cameraSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private PID nuzzlePID = new PID(0.1, 0.0, 0, 0.8, -0.8, 0.3, null);

    public NuzzleUpV2Command(PID nuzzlePID, DistanceSubsystem distanceSubsystem, SwerveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.distanceSubsystem = distanceSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        this.leftRightVelocity = 0.0;
        this.forwardVelocity = 0.0;
    }


    @Override
    public void initialize() {
        super.initialize();

        nuzzlePID.setGoal(0);
    }

    @Override
    public void execute() {
        super.execute();
        if (distanceSubsystem.shouldMoveLeft()){
            //leftRightVelocity = whatever velocity moves it left
        }
        else if (distanceSubsystem.shouldMoveRight()) {
            //leftRightVelocity = whatever velocity moves it right
        }
        //forwardVelocity = however we obtain the distance from the reef from the april tag converted to velocity
        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(forwardVelocity, leftRightVelocity, 0));
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
