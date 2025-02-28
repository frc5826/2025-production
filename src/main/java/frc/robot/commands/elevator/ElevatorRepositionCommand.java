package frc.robot.commands.elevator;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorRepositionCommand extends ElevatorPositionCommand {

    private double offset;

    public ElevatorRepositionCommand(ElevatorSubsystem elevatorSubsystem, double offset, ElevatorSubsystem.LevelTarget levelTarget) {
        super(elevatorSubsystem, 0, levelTarget);
        this.offset = offset;
    }

    @Override
    public void initialize() {
        double targetPosition = elevatorSubsystem.getDesiredPosition() + offset;
        super.initialize();
        elevatorSubsystem.setDesiredPosition(targetPosition, levelTarget);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();    }
}
