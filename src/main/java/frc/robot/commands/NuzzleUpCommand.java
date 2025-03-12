//package frc.robot.commands;
//
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import frc.robot.math.PID;
//import frc.robot.sensors.UltrasonicAN;
//import frc.robot.subsystems.DistanceSubsystem;
//import frc.robot.subsystems.SwerveSubsystem;
//
//public class NuzzleUpCommand extends LoggedCommand {
//
//    //This command is called to rub against the reef after Q's stuff gets us close to the reef
//    //Hence the name "Nuzzle Up"
//    //This command will ask April Tags and Ultrasonics about the robots rotation and distance
//    //Once we have these amounts we will change the robots rotation and distance until aligned
//
//    private DistanceSubsystem distanceSubsystem;
//    private SwerveSubsystem swerveSubsystem;
//    private PID nuzzlePID = new PID(0.1, 0.0, 0, 0.8, -0.8, 0.3, null);
//
//    public NuzzleUpCommand(UltrasonicAN ultrasonic1, UltrasonicAN ultrasonic2, PID nuzzlePID, DistanceSubsystem distanceSubsystem, SwerveSubsystem swerveSubsystem){
//        this.swerveSubsystem = swerveSubsystem;
//        this.distanceSubsystem = distanceSubsystem;
//    }
//
//    @Override
//    public void initialize() {
//        super.initialize();
//
//        nuzzlePID.setGoal(0);
//    }
//
//    @Override
//    public void execute() {
//        super.execute();
//        if (distanceSubsystem.ultrasonicsDontEqual()) {
//            swerveSubsystem.driveRobotOriented(new ChassisSpeeds()); //TODO vro IDK what to do here
//        }
//    }
//
//    @Override
//    public boolean isFinished() {
//        return super.isFinished();
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        super.end(interrupted);
//    }
//}
//
