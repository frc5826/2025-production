// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.Dumb;
import frc.robot.commands.commandgroups.*;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerReWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.elevator.ElevatorRepositionCommand;
import frc.robot.commands.swerve.CrabWalkCommand;
import frc.robot.commands.swerve.DriveButtonCommand;
import frc.robot.commands.swerve.pathing.PathOffsetThenAccurateCommand;
import frc.robot.commands.swerve.pathing.PathToCommand;
import frc.robot.positioning.Orientation;
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
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.localization.Localization;

import java.util.ArrayList;

import static frc.robot.Constants.*;

public class RobotContainer {

    public final CameraSubsystem cameraSubsystem = new CameraSubsystem();

    public final Localization localization = new Localization(cameraSubsystem);
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(localization);

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public final CoralizerSubsystem coralizerSubsystem = new CoralizerSubsystem();

    DeferredLevelCommand deferredLevelCommand = new DeferredLevelCommand(elevatorSubsystem, coralizerSubsystem, swerveSubsystem);

    private Field2d field;

    public RobotContainer() {
        DataLogManager.start("/U/logs");
        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, new TeleopDriveCommand(swerveSubsystem));

        // Setup button bindings
        bindXbox();
        bindBoard();

        //Create elastic tabs
        field = new Field2d();
        setupFieldTab();

        setupAutoTab();
    }

    public void prePeriodic(boolean teleop) {

        SmartDashboard.putNumber("Adjusted angle", swerveSubsystem.getAdjustedIMUContinuousAngle().getDegrees());
        SmartDashboard.putNumber("Not adjusted angle", swerveSubsystem.getIMUContinuousAngle().getDegrees());

        if (!swerveSubsystem.getOrientation().equals(FieldOrientation.getOrientation())) {
            swerveSubsystem.setOrientation(FieldOrientation.getOrientation());
        }

        //if (teleop) {
            localization.move();
            localization.measure(swerveSubsystem);
            updateField();
        //}

    }

    public void postPeriodic() {

    }

    public Command getAutoCommand() {
        return new Dumb(swerveSubsystem);
    }

    private void bindXbox() {
        //new Trigger(cXbox::getBackButtonPressed).onTrue(new InstantCommand(swerveSubsystem::resetDriveGyro)); //TODO zero gyro command

        new Trigger(() -> cXbox.getRightTriggerAxis() > 0.5).onTrue(new InstantCommand(() -> swerveSubsystem.setSpeedMultiplier(true)));
        new Trigger(() -> cXbox.getRightTriggerAxis() < 0.5).onTrue(new InstantCommand(() -> swerveSubsystem.setSpeedMultiplier(false)));

        new Trigger(() -> cXbox.getPOV() == 0).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.FRONT, 0.35, swerveSubsystem));
        new Trigger(() -> cXbox.getPOV() == 90).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.RIGHT, 0.35, swerveSubsystem));
        new Trigger(() -> cXbox.getPOV() == 180).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.BACK, 0.35, swerveSubsystem));
        new Trigger(() -> cXbox.getPOV() == 270).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.LEFT, 0.35, swerveSubsystem));

        new Trigger(cXbox::getRightBumperButton).whileTrue(new DriveButtonCommand(new ChassisSpeeds(0, 0, -0.7), swerveSubsystem));
        new Trigger(cXbox::getLeftBumperButton).whileTrue(new DriveButtonCommand(new ChassisSpeeds(0, 0, 0.7), swerveSubsystem));

        //tuning path pid
        PathConstraints constraints = new PathConstraints(1.25, 2, Math.PI * 2,  Math.PI * 2);
//        new Trigger(cXbox::getAButton).whileTrue(new PathToCommand(new Pose2d(1.25, 1.5, new Rotation2d(Math.PI)), 0, constraints, swerveSubsystem));
//        new Trigger(cXbox::getBButton).whileTrue(new PathToCommand(new Pose2d(1.25, 4, new Rotation2d(Math.PI)), 0, constraints, swerveSubsystem));
        new Trigger(cXbox::getAButton).onTrue(deferredLevelCommand);
    }

    //TODO set real constraints and different constraints variable for Source Pickup :)
    private void bindBoard() {
        PathConstraints constraints = new PathConstraints(1.25, 2, Math.PI * 2,  Math.PI * 2);
        //For Buttons 0-11, Starts at top left white button and goes clockwise around
        new Trigger(() -> cButtonBoard.getButton(0)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefH(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(1)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefG(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(2)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefF(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(3)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefE(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(4)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefD(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(5)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefC(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(6)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefB(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(7)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefA(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(8)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefL(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(9)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefK(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(10)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefJ(), constraints, 0.75, true, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(11)).whileTrue(new PathOffsetThenAccurateCommand(FieldOrientation.getOrientation().getReefI(), constraints, 0.75, true, swerveSubsystem));
        //new Trigger(() -> cButtonBoard.getButtonPressed(11)).onTrue(new AutoGroundPickupCommand(elevatorSubsystem, coralizerSubsystem)); //TODO add a button for this

        //For Buttons 12-15, Starts at top white button and goes straight down
        new Trigger(() -> cButtonBoard.getButtonPressed(12)).onTrue(new L4CommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButtonPressed(12)).onTrue(new InstantCommand(() -> {deferredLevelCommand.setTarget(DeferredLevelCommand.DeferredLevel.L4);}));
        new Trigger(() -> cButtonBoard.getButtonPressed(13)).onTrue(new L3CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cButtonBoard.getButtonPressed(13)).onTrue(new InstantCommand(() -> {deferredLevelCommand.setTarget(DeferredLevelCommand.DeferredLevel.L3);}));
        new Trigger(() -> cButtonBoard.getButtonPressed(14)).onTrue(new L2CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cButtonBoard.getButtonPressed(14)).onTrue(new InstantCommand(() -> {deferredLevelCommand.setTarget(DeferredLevelCommand.DeferredLevel.L2);}));
        new Trigger(() -> cButtonBoard.getButtonPressed(15)).onTrue(new L1CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cButtonBoard.getButtonPressed(15)).onTrue(new InstantCommand(() -> {deferredLevelCommand.setTarget(DeferredLevelCommand.DeferredLevel.L1);}));
        //For Buttons 16-18, Starts at top right black button and goes left
        new Trigger(() -> cButtonBoard.getButtonPressed(16)).onTrue(new DropoffCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(17)).onTrue(new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(18)).onTrue(new SourceCommandGroup(elevatorSubsystem, coralizerSubsystem));
        //For Buttons 19-21, Starts at middle right red button and goes left
        new Trigger(() -> cButtonBoard.getButton(19)).whileTrue(new AlignSourceCommandGroup(FieldOrientation.getOrientation().getCoralStationLB(), constraints, swerveSubsystem, elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButton(20)).whileTrue(new AlignSourceCommandGroup(FieldOrientation.getOrientation().getCoralStationRB(), constraints, swerveSubsystem, elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButton(21)).whileTrue(new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.OUT));
        //For Buttons 22-24, Starts at bottom right white button and goes left
        new Trigger(() -> cButtonBoard.getButtonPressed(22)).onTrue(new DealgifyL2CommandGroup(elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(23)).onTrue(new DealgifyL3CommandGroup(elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButton(24)).whileTrue(new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.IN));
        //Ground pickup
        //new Trigger(() -> cButtonBoard.getButtonPressed(25)).onTrue(new AutoGroundPickupCommand(elevatorSubsystem, coralizerSubsystem));
    }

    public void initZeroGyro() {
        swerveSubsystem.zeroOdoGyro(Math.toRadians(FieldOrientation.getOrientation().getStartOrientation()));
    }

    private void setupAutoTab() {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

        SendableChooser<Pose2d> firstReef = new SendableChooser<Pose2d>();
        SendableChooser<Pose2d> secondReef = new SendableChooser<Pose2d>();
        SendableChooser<Boolean> dumb = new SendableChooser<Boolean>();

        Orientation orientation = FieldOrientation.getOrientation();

        dumb.addOption("Yes dumb", true);
        dumb.addOption("No dumb", false);

        autoTab.add("Dumb?", firstReef)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withSize(2, 2)
                .withPosition(0, 0);

        firstReef.addOption("A", orientation.getReefA());
        firstReef.addOption("B", orientation.getReefB());
        firstReef.addOption("C", orientation.getReefC());
        firstReef.addOption("D", orientation.getReefD());
        firstReef.addOption("E", orientation.getReefE());
        firstReef.addOption("F", orientation.getReefF());
        firstReef.addOption("G", orientation.getReefG());
        firstReef.addOption("H", orientation.getReefH());
        firstReef.addOption("I", orientation.getReefI());
        firstReef.addOption("J", orientation.getReefJ());
        firstReef.addOption("K", orientation.getReefK());
        firstReef.addOption("L", orientation.getReefL());

        autoTab.add("First reef target", firstReef)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withSize(2, 2)
                .withPosition(2, 0);

        secondReef.addOption("Nothing", new Pose2d());
        secondReef.addOption("A", orientation.getReefA());
        secondReef.addOption("B", orientation.getReefB());
        secondReef.addOption("C", orientation.getReefC());
        secondReef.addOption("D", orientation.getReefD());
        secondReef.addOption("E", orientation.getReefE());
        secondReef.addOption("F", orientation.getReefF());
        secondReef.addOption("G", orientation.getReefG());
        secondReef.addOption("H", orientation.getReefH());
        secondReef.addOption("I", orientation.getReefI());
        secondReef.addOption("J", orientation.getReefJ());
        secondReef.addOption("K", orientation.getReefK());
        secondReef.addOption("L", orientation.getReefL());

        autoTab.add("Second reef target", secondReef)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withSize(2, 2)
                .withPosition(4, 0);
    }

    public void updateField() {
        field.setRobotPose(localization.getPose());
    }

    private void setupFieldTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("field");

        tab.add("PDH", new PowerDistribution(1, PowerDistribution.ModuleType.kRev));

        field = new Field2d();
        tab.add(field)
                .withPosition(4,0)
                .withSize(4,3);

        //Filtered position
        ShuffleboardLayout position = tab.getLayout("Filtered position", BuiltInLayouts.kList)
                .withPosition(2,0)
                .withSize(2,3);

        position.addDouble("Robot X", ()-> localization.getPose().getX());
        position.addDouble("Robot Y", ()-> localization.getPose().getY());
        position.addDouble("Robot rotation", ()-> localization.getPose().getRotation().getDegrees());


//        //Filter inputs
//        ShuffleboardLayout inputs = tab.getLayout("Filter inputs", BuiltInLayouts.kList)
//                .withPosition(0, 0).withSize(2, 4);
//
////        if (swerveSubsystem.getFieldAcc().isPresent()) {
////            inputs.add("Acceleration", swerveSubsystem.getFieldAcc().get());
////        }
//        inputs.add("Velocity", swerveSubsystem.getOdoFieldVel());
//        inputs.add("Adjusted gyro angle", swerveSubsystem.getAdjustedIMUContinuousAngle());
//        inputs.add("Cam pose", cameraSubsystem.getCameraMeasurements());
    }

}
