package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggedCommand;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Elevator.cElevatorDeadband;

public class ElevatorPositionCommand extends LoggedCommand {

    protected ElevatorSubsystem elevatorSubsystem;
    protected DoubleSupplier position;
    protected ReefPosition.ReefLevel levelTarget;
    protected Timer timeoutTimer;


    public ElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier position, ReefPosition.ReefLevel levelTarget){

        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;
        this.levelTarget = levelTarget;

        addRequirements(elevatorSubsystem);

        timeoutTimer = new Timer();
    }

    public ElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, double position, ReefPosition.ReefLevel levelTarget) {
        this(elevatorSubsystem, () -> position, levelTarget);
    }

    @Override
    public void initialize() {
        super.initialize();

        elevatorSubsystem.setDesiredPosition(position.getAsDouble());
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
        super.end(interrupted);
        timeoutTimer.reset();
    }
}
