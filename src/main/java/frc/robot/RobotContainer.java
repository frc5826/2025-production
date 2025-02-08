// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.commandgroups.*;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerReWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.elevator.ElevatorRepositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.pathing.PathFindCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.commands.swerve.TeleopDriveCommand;
import frc.robot.localization.Localization;
import frc.robot.positioning.FieldOrientation;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.cJoystick;
import static frc.robot.Constants.cXbox;

public class RobotContainer
{

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public final ElevatorPositionCommand elevatorHomeCommand = new ElevatorPositionCommand(elevatorSubsystem, 0);

    public final ElevatorRepositionCommand elevatorRepositionCommandDown = new ElevatorRepositionCommand(elevatorSubsystem, -0.1);
    public final ElevatorRepositionCommand elevatorRepositionCommandUp = new ElevatorRepositionCommand(elevatorSubsystem, 0.1);
    public final ElevatorRepositionCommand elevatorRepositionCommandBigDown = new ElevatorRepositionCommand(elevatorSubsystem, -0.5);
    public final ElevatorRepositionCommand elevatorRepositionCommandBigUp = new ElevatorRepositionCommand(elevatorSubsystem, 0.5);

    public final CoralizerSubsystem coralizerSubsystem = new CoralizerSubsystem();

    public final CoralizerIntakeCommand coralizerInCommand = new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.IN);
    public final CoralizerIntakeCommand coralizerOutCommand = new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.OUT);
    public final CoralizerIntakeCommand coralizerShootCommand = new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.SHOOT);
    public final CoralizerReWristCommand coralizerWristCommandUp = new CoralizerReWristCommand(coralizerSubsystem, 1);
    public final CoralizerReWristCommand coralizerWristCommandDown = new CoralizerReWristCommand(coralizerSubsystem, -1);

    public final L1CommandGroup l1CommandGroup = new L1CommandGroup(elevatorSubsystem, coralizerSubsystem);
    public final L2CommandGroup l2CommandGroup = new L2CommandGroup(elevatorSubsystem, coralizerSubsystem);
    public final L3CommandGroup l3CommandGroup = new L3CommandGroup(elevatorSubsystem, coralizerSubsystem);
    public final L4CommandGroup l4CommandGroup = new L4CommandGroup(elevatorSubsystem, coralizerSubsystem);
    public final L4DropoffCommandGroup dropoffCommandGroup = new L4DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem);
    public final SourceCommandGroup sourceCommandGroup = new SourceCommandGroup(elevatorSubsystem, coralizerSubsystem);
    public final DealgifyCommandGroup dealgifyCommandGroup = new DealgifyCommandGroup(elevatorSubsystem, coralizerSubsystem);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public final Localization localization = new Localization();

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(localization);

    public RobotContainer()
    {
        DataLogManager.start("/U/logs");
        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem,new TeleopDriveCommand(swerveSubsystem));

        // Setup button bindings
        bindXbox();
        bindBoard();
    }

    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        new Trigger(cJoystick::getTrigger).onTrue(dropoffCommandGroup);

        new Trigger(() -> cJoystick.getRawButton(2)).whileTrue(sourceCommandGroup);

        new Trigger(() -> cJoystick.getRawButton(3)).onTrue(coralizerWristCommandDown);
        new Trigger(() -> cJoystick.getRawButton(5)).onTrue(coralizerWristCommandUp);
        new Trigger(() -> cJoystick.getRawButton(4)).whileTrue(coralizerInCommand);
        new Trigger(() -> cJoystick.getRawButton(6)).whileTrue(coralizerOutCommand);

        new Trigger(() -> cJoystick.getRawButton(7)).onTrue(dealgifyCommandGroup);

        new Trigger(() -> cJoystick.getRawButton(8)).whileTrue(coralizerShootCommand);

        new Trigger(() -> cJoystick.getRawButton(9)).onTrue(l3CommandGroup);
        new Trigger(() -> cJoystick.getRawButton(10)).onTrue(l4CommandGroup);
        new Trigger(() -> cJoystick.getRawButton(11)).onTrue(l1CommandGroup);
        new Trigger(() -> cJoystick.getRawButton(12)).onTrue(l2CommandGroup);
    }

    public void prePeriodic(boolean teleop) {

        if(teleop) {
            localization.move();
            localization.measure(swerveSubsystem);
            localization.updateField();
        }

    }

    public void postPeriodic() {

    }

    private void bindXbox() {
        new Trigger(cXbox::getBackButtonPressed).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
        new Trigger(cXbox::getStartButtonPressed).onTrue(new InstantCommand(this::initZeroGyro)); //TODO test

        PathConstraints constraints = new PathConstraints(1, 2, Math.PI * 3, Math.PI * 3);
        new Trigger(cXbox::getAButton).whileTrue(new PathToCommand(FieldOrientation.getOrientation().getReefA(), 0, constraints, swerveSubsystem));
        new Trigger(cXbox::getBButton).whileTrue(new PathToCommand(FieldOrientation.getOrientation().getCoralStationRB(), 0, constraints, swerveSubsystem));
        new Trigger(cXbox::getYButton).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefF(), constraints, swerveSubsystem));
        new Trigger(cXbox::getXButton).whileTrue(new PathToCommand(FieldOrientation.getOrientation().getReefC(), 0, constraints, swerveSubsystem));
    }

    private void bindBoard() {

    }

    //Called in auto init to give the cameras time to localize us
    public void initZeroGyro() {
        swerveSubsystem.setGyroOffset(localization.getCameraPose().getRotation().getRadians());
    }
}
