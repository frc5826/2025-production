package frc.robot.commands.elevator;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ElevatorSubsystem;

import java.rmi.dgc.Lease;

import static frc.robot.Constants.Elevator.cElevatorDeadband;

public class ElevatorRepositionCommand extends LoggedCommand {

    private ElevatorSubsystem elevatorSubsystem;
    private ElevatorSubsystem.LevelTarget levelTarget;
    private double position;


    public ElevatorRepositionCommand(ElevatorSubsystem elevatorSubsystem, double reposition, ElevatorSubsystem.LevelTarget levelTarget){

        this.elevatorSubsystem = elevatorSubsystem;
        this.position = reposition;
        this.levelTarget = levelTarget;

        addRequirements(elevatorSubsystem);

    }

    @Override
    public void initialize() {
        super.initialize();

        elevatorSubsystem.setDesiredPosition(elevatorSubsystem.getDesiredPosition() + position, levelTarget);

    }

    @Override
    public boolean isFinished() {

        return Math.abs(elevatorSubsystem.getDesiredPosition() - elevatorSubsystem.getPos()) <= cElevatorDeadband;

    }
}
