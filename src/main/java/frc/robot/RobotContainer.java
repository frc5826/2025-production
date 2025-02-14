// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.commandgroups.*;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerReWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.elevator.ElevatorRepositionCommand;
import frc.robot.commands.swerve.CrabWalkCommand;
import frc.robot.commands.swerve.DriveButtonCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.swerve.pathing.AccuratePathCommand;
import frc.robot.commands.swerve.pathing.PathFindCommand;
import frc.robot.positioning.FieldOrientation;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.TeleopDriveCommand;
import frc.robot.localization.Localization;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.*;

public class RobotContainer {

    public final CameraSubsystem cameraSubsystem = new CameraSubsystem();

    public final Localization localization = new Localization(cameraSubsystem);
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(localization);

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
    public final L4CommandGroup l4CommandGroup = new L4CommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem);
    public final L4DropoffCommandGroup L4DropoffCommandGroup = new L4DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem);
    public final L3L2DropoffCommandGroup L3L2DropoffCommandGroup = new L3L2DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem);
    public final SourceCommandGroup sourceCommandGroup = new SourceCommandGroup(elevatorSubsystem, coralizerSubsystem);
    public final DealgifyL2CommandGroup dealgifyL2CommandGroup = new DealgifyL2CommandGroup(elevatorSubsystem, coralizerSubsystem);

    public RobotContainer() {
        DataLogManager.start("/U/logs");
        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, new TeleopDriveCommand(swerveSubsystem));

        // Setup button bindings
        bindXbox();
        bindBoard();
    }

    public void prePeriodic(boolean teleop) {

        if (teleop) {
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

        new Trigger(() -> cXbox.getRightTriggerAxis() > 0.5).onTrue(new InstantCommand(() -> swerveSubsystem.setSpeedMultiplier(true)));
        new Trigger(() -> cXbox.getRightTriggerAxis() < 0.5).onTrue(new InstantCommand(() -> swerveSubsystem.setSpeedMultiplier(false)));

        new Trigger(() -> cXbox.getPOV() == 0).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.FRONT, 0.3, swerveSubsystem));
        new Trigger(() -> cXbox.getPOV() == 90).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.RIGHT, 0.3, swerveSubsystem));
        new Trigger(() -> cXbox.getPOV() == 180).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.BACK, 0.3, swerveSubsystem));
        new Trigger(() -> cXbox.getPOV() == 270).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.LEFT, 0.3, swerveSubsystem));

        new Trigger(cXbox::getRightBumperButton).whileTrue(new DriveButtonCommand(new ChassisSpeeds(0, 0, -0.7), swerveSubsystem));
        new Trigger(cXbox::getLeftBumperButton).whileTrue(new DriveButtonCommand(new ChassisSpeeds(0, 0, 0.7), swerveSubsystem));

    }

    //TODO set real constraints and different constraints variable for Source Pickup :)
    private void bindBoard() {
        PathConstraints constraints = new PathConstraints(1.25, 2, Math.PI * 2,  Math.PI * 2);
        //For Buttons 0-11, Starts at top left white button and goes clockwise around
        new Trigger(() -> cButtonBoard.getButton(0)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefH(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefH(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(1)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefG(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefG(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(2)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefF(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefF(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(3)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefE(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefE(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(4)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefD(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefD(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(5)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefC(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefC(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(6)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefB(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefB(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(7)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefA(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefA(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(8)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefL(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefL(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(9)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefK(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefK(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(10)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefJ(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefJ(), false, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(11)).whileTrue(new PathFindCommand(FieldOrientation.getOrientation().getReefI(), constraints, swerveSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getReefI(), false, cameraSubsystem, swerveSubsystem)));
        //For Buttons 12-15, Starts at top white button and goes straight down
        new Trigger(() -> cButtonBoard.getButtonPressed(12)).onTrue(new L4CommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(13)).onTrue(new L3CommandGroup(elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(14)).onTrue(new L2CommandGroup(elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(15)).onTrue(new L1CommandGroup(elevatorSubsystem, coralizerSubsystem));
        //For Buttons 16-18, Starts at top right black button and goes left
        new Trigger(() -> cButtonBoard.getButtonPressed(16)).onTrue(new DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(17)).onTrue(new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(18)).onTrue(new DealgifyL2CommandGroup(elevatorSubsystem, coralizerSubsystem));
        //For Buttons 19-21, Starts at middle right red button and goes left
        new Trigger(() -> cButtonBoard.getButton(19)).whileTrue(new AllignSourceCommandGroup(FieldOrientation.getOrientation().getCoralStationLB(), constraints, swerveSubsystem, elevatorSubsystem, coralizerSubsystem, cameraSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getCoralStationLB(), true, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(20)).whileTrue(new AllignSourceCommandGroup(FieldOrientation.getOrientation().getCoralStationRB(), constraints, swerveSubsystem, elevatorSubsystem, coralizerSubsystem, cameraSubsystem).andThen(new AccuratePathCommand(FieldOrientation.getOrientation().getCoralStationRB(), true, cameraSubsystem, swerveSubsystem)));
        new Trigger(() -> cButtonBoard.getButton(21)).whileTrue(new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.OUT));
        //For Buttons 22-24, Starts at bottom right white button and goes left
        new Trigger(() -> cButtonBoard.getButton(22)).whileTrue(new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.IN));
        new Trigger(() -> cButtonBoard.getButtonPressed(23)).onTrue(new DealgifyL3CommandGroup(elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(24)).onTrue(new SourceCommandGroup(elevatorSubsystem, coralizerSubsystem));
        //Cancel current command
//        new Trigger(() -> cButtonBoard.getButtonPressed(25)).onTrue(() -> CommandScheduler.getInstance().cancelAll());
    }

    //Called in auto init to give the cameras time to localize us
    public void initZeroGyro() {

        if (DriverStation.getAlliance().isPresent()) {
            swerveSubsystem.setGyroOffset(
                    localization.getCameraPose().getRotation().getRadians()
                            + (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0 : Math.PI));
//            localization.
        } else {
            System.err.println("Ah heck, no alliance found! Gyro is not zeroed :(");
        }

    }
}
