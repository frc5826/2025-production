package frc.robot.commands.elevator;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.Elevator.cElevatorDeadband;

public class ElevatorRepositionCommand extends LoggedCommand {

    private ElevatorSubsystem elevatorSubsystem;
    private double position;


    public ElevatorRepositionCommand(ElevatorSubsystem elevatorSubsystem, double reposition){

        this.elevatorSubsystem = elevatorSubsystem;
        this.position = reposition;

        addRequirements(elevatorSubsystem);

    }

    @Override
    public void initialize() {
        super.initialize();

        elevatorSubsystem.setDesiredPosition(elevatorSubsystem.getDesiredPos() + position);

    }

    @Override
    public boolean isFinished() {

        return true; //Math.abs(elevatorSubsystem.getDesiredPos() - elevatorSubsystem.getPos()) <= cElevatorDeadband;

    }
}
