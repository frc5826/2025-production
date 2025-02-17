package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.Elevator.cElevatorDeadband;

public class ElevatorPositionCommand extends LoggedCommand {

    private ElevatorSubsystem elevatorSubsystem;
    private double position;
    private Timer timeoutTimer;


    public ElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, double position){

        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;

        addRequirements(elevatorSubsystem);

        timeoutTimer = new Timer();
    }

    @Override
    public void initialize() {
        super.initialize();

        elevatorSubsystem.setDesiredPosition(position);
        timeoutTimer.reset();
        timeoutTimer.start();
    }

    @Override
    public boolean isFinished() {
        if (timeoutTimer.get() >= 3){return true;}
        return Math.abs(elevatorSubsystem.getDesiredPosition() - elevatorSubsystem.getPos()) <= cElevatorDeadband;

    }

    @Override
    public void end(boolean interrupted) {
        timeoutTimer.reset();
    }
}
