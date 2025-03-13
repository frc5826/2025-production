// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.AutoCommandGroup;
import frc.robot.commands.autos.Dumb;
import frc.robot.commands.commandgroups.*;
import frc.robot.commands.commandgroups.algae.DealgifyL2CommandGroup;
import frc.robot.commands.commandgroups.algae.DealgifyL3CommandGroup;
import frc.robot.commands.commandgroups.SourceCommandGroup;
import frc.robot.commands.commandgroups.dropoff.DropoffCommandGroup;
import frc.robot.commands.commandgroups.reef.*;
import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.swerve.drivercontrol.CrabWalkCommand;
import frc.robot.commands.swerve.drivercontrol.DriveButtonCommand;
import frc.robot.commands.swerve.pathing.*;
import frc.robot.positioning.ReefPosition;
import frc.robot.subsystems.*;
import frc.robot.positioning.FieldOrientation;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.drivercontrol.TeleopDriveCommand;
import frc.robot.localization.Localization;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static frc.robot.Constants.*;

public class RobotContainer {

    public final CameraSubsystem cameraSubsystem = new CameraSubsystem();

    public final Localization localization = new Localization(cameraSubsystem);
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(localization);

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public final CoralizerSubsystem coralizerSubsystem = new CoralizerSubsystem();

    public final ReefTargeting reefTargeting = new ReefTargeting(swerveSubsystem);

    private Field2d field;

    List<SendableChooser<Pose2d>> reefLocations = new ArrayList<>();
    List<SendableChooser<ReefPosition.ReefLevel>> reefLevels = new ArrayList<>();
    SendableChooser<Boolean> dumb = new SendableChooser<>();
    SendableChooser<Supplier<Pose2d>> autoStationPosition = new SendableChooser<>();
    SendableChooser<Boolean> pathCoral = new SendableChooser<>();

    public RobotContainer() {
        DataLogManager.start("/U/logs");
//        StringLogEntry entry = new StringLogEntry(DataLogManager.getLog(), "/ntlog");
//        NetworkTableInstance.getDefault().addLogger(0, 100,
//                event -> entry.append(event.logMessage.filename + ":" + event.logMessage.line + ":" + event.logMessage.message));
        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, new TeleopDriveCommand(swerveSubsystem));

//        new DriverCamera(); Removed usb camera

        // Setup button bindings
        bindXbox();
        bindBoard();

        //Create elastic tabs
        field = new Field2d();
        setupFieldTab();

        setupAutoTab();
    }

    public void prePeriodic() {
        SmartDashboard.putNumber("Adjusted angle", swerveSubsystem.getAdjustedIMUContinuousAngle().getDegrees());
        SmartDashboard.putNumber("Not adjusted angle", swerveSubsystem.getIMUContinuousAngle().getDegrees());

        if (!swerveSubsystem.getOrientation().equals(FieldOrientation.getOrientation())) {
            swerveSubsystem.setOrientation(FieldOrientation.getOrientation());
        }
        
        localization.move();
        localization.measure(swerveSubsystem);
        updateField();
    }

    public void postPeriodic() {

    }

    private void bindXbox() {
        //new Trigger(cXbox::getBackButtonPressed).onTrue(new InstantCommand(swerveSubsystem::resetDriveGyro)); //TODO zero gyro command

        new Trigger(() -> cXbox.getRightTriggerAxis() > 0.5).onTrue(new InstantCommand(() -> swerveSubsystem.setSpeedMultiplier(true)));
        new Trigger(() -> cXbox.getRightTriggerAxis() < 0.5).onTrue(new InstantCommand(() -> swerveSubsystem.setSpeedMultiplier(false)));

        new Trigger(() -> cXbox.getPOV() == 0).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.FRONT, swerveSubsystem));
        new Trigger(() -> cXbox.getPOV() == 90).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.RIGHT, swerveSubsystem));
        new Trigger(() -> cXbox.getPOV() == 180).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.BACK, swerveSubsystem));
        new Trigger(() -> cXbox.getPOV() == 270).whileTrue(new CrabWalkCommand(CrabWalkCommand.Direction.LEFT, swerveSubsystem));

        new Trigger(cXbox::getRightBumperButton).whileTrue(new DriveButtonCommand(new ChassisSpeeds(0, 0, -0.7), swerveSubsystem));
        new Trigger(cXbox::getLeftBumperButton).whileTrue(new DriveButtonCommand(new ChassisSpeeds(0, 0, 0.7), swerveSubsystem));

        new Trigger(cXbox::getAButton).whileTrue(new ScoreCommandGroup(reefTargeting, swerveSubsystem, elevatorSubsystem, coralizerSubsystem));
        new Trigger(cXbox::getXButton).whileTrue(new AlignReefCommand(reefTargeting, swerveSubsystem));
        new Trigger(cXbox::getBButton).onTrue(new ReefCommand(reefTargeting, elevatorSubsystem, coralizerSubsystem));

        //new Trigger(cXbox::getAButton).whileTrue(new AlignReefCameraCommand(cameraSubsystem, swerveSubsystem));
        //new Trigger(cXbox::getBButtonPressed).onTrue(new MovingHeightCommandGroup(elevatorSubsystem, coralizerSubsystem));

        //For testing cuz the button board is being poopy
//        new Trigger(() -> cJoystick.getRawButtonPressed(12)).onTrue(new L4CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cJoystick.getRawButtonPressed(11)).onTrue(new L3CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cJoystick.getRawButtonPressed(10)).onTrue(new L2CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cJoystick.getRawButtonPressed(9)).onTrue(new L1CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cJoystick.getRawButtonPressed(8)).onTrue(new SourceCommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cJoystick.getRawButtonPressed(7)).onTrue(new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cJoystick.getRawButton(3)).whileTrue(new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.OUT));
//        new Trigger(() -> cJoystick.getRawButton(4)).whileTrue(new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.IN));
//
//        new Trigger(() -> cJoystick.getRawButton(5)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefH(), FieldOrientation.getOrientation().getReefSideGH(), swerveSubsystem, elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cJoystick.getRawButton(6)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefG(), FieldOrientation.getOrientation().getReefSideGH(), swerveSubsystem, elevatorSubsystem, coralizerSubsystem));
    }

    private void bindBoard() {
//        new Trigger(() -> cButtonBoard.getButton(0)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefH(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(1)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefG(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(2)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefF(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(3)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefE(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(5)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefC(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(4)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefD(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(6)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefB(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(7)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefA(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(8)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefL(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(9)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefK(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(10)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefJ(), constraints, swerveSubsystem));
//        new Trigger(() -> cButtonBoard.getButton(11)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefI(), constraints, swerveSubsystem));

        //Set pose goals
        new Trigger(() -> cButtonBoard.getButtonPressed(0)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefH())));
        new Trigger(() -> cButtonBoard.getButtonPressed(1)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefG())));
        new Trigger(() -> cButtonBoard.getButtonPressed(2)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefF())));
        new Trigger(() -> cButtonBoard.getButtonPressed(3)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefE())));
        new Trigger(() -> cButtonBoard.getButtonPressed(4)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefD())));
        new Trigger(() -> cButtonBoard.getButtonPressed(5)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefC())));
        new Trigger(() -> cButtonBoard.getButtonPressed(6)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefB())));
        new Trigger(() -> cButtonBoard.getButtonPressed(7)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefA())));
        new Trigger(() -> cButtonBoard.getButtonPressed(8)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefL())));
        new Trigger(() -> cButtonBoard.getButtonPressed(9)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefK())));
        new Trigger(() -> cButtonBoard.getButtonPressed(10)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefJ())));
        new Trigger(() -> cButtonBoard.getButtonPressed(11)).onTrue(new InstantCommand(() -> reefTargeting.updatePose(FieldOrientation.getOrientation().getReefI())));

        //testing new align
        //new Trigger(() -> cButtonBoard.getButton(0)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefH(), FieldOrientation.getOrientation().getReefSideGH(), swerveSubsystem, elevatorSubsystem, coralizerSubsystem));
        //new Trigger(() -> cButtonBoard.getButton(1)).whileTrue(new AlignReefCommand(FieldOrientation.getOrientation().getReefG(), FieldOrientation.getOrientation().getReefSideGH(), swerveSubsystem, elevatorSubsystem, coralizerSubsystem));

        //For Buttons 12-15, Starts at top white button and goes straight down
        new Trigger(() -> cButtonBoard.getButtonPressed(12)).onTrue(new InstantCommand(() -> reefTargeting.updateLevel(ReefPosition.ReefLevel.L4)));
        new Trigger(() -> cButtonBoard.getButtonPressed(13)).onTrue(new InstantCommand(() -> reefTargeting.updateLevel(ReefPosition.ReefLevel.L3)));
        new Trigger(() -> cButtonBoard.getButtonPressed(14)).onTrue(new InstantCommand(() -> reefTargeting.updateLevel(ReefPosition.ReefLevel.L2)));
        new Trigger(() -> cButtonBoard.getButtonPressed(15)).onTrue(new InstantCommand(() -> reefTargeting.updateLevel(ReefPosition.ReefLevel.L1)));
//        new Trigger(() -> cButtonBoard.getButtonPressed(12)).onTrue(new L4CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cButtonBoard.getButtonPressed(13)).onTrue(new L3CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cButtonBoard.getButtonPressed(14)).onTrue(new L2CommandGroup(elevatorSubsystem, coralizerSubsystem));
//        new Trigger(() -> cButtonBoard.getButtonPressed(15)).onTrue(new L1CommandGroup(elevatorSubsystem, coralizerSubsystem));
        //For Buttons 16-18, Starts at top right black button and goes left
        new Trigger(() -> cButtonBoard.getButtonPressed(16)).onTrue(new DropoffCommandGroup(reefTargeting, elevatorSubsystem, coralizerSubsystem, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(17)).onTrue(new HomeCommandGroup(elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(18)).onTrue(new SourceCommandGroup(elevatorSubsystem, coralizerSubsystem));
        //For Buttons 19-21, Starts at middle right red button and goes left
        new Trigger(() -> cButtonBoard.getButton(19)).whileTrue(new AlignSourceCommandGroup(FieldOrientation.getOrientation().getCoralStationLB(), swerveSubsystem, elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButton(20)).whileTrue(new AlignSourceCommandGroup(FieldOrientation.getOrientation().getCoralStationRB(), swerveSubsystem, elevatorSubsystem, coralizerSubsystem));
        new Trigger(() -> cButtonBoard.getButton(21)).whileTrue(new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.OUT));
        //For Buttons 22-24, Starts at bottom right white button and goes left
        new Trigger(() -> cButtonBoard.getButtonPressed(22)).onTrue(new DealgifyL2CommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButtonPressed(23)).onTrue(new DealgifyL3CommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem));
        new Trigger(() -> cButtonBoard.getButton(24)).whileTrue(new CoralizerIntakeCommand(coralizerSubsystem, CoralizerIntakeCommand.IntakeDirection.IN));
    }

    public void autoInit() {
        swerveSubsystem.zeroOdoGyro(Math.toRadians(FieldOrientation.getOrientation().getStartOrientation()));
        swerveSubsystem.setOrientation(FieldOrientation.getOrientation());
        coralizerSubsystem.startWristTimer();
    }

    private void setupAutoTab(){
//        Orientation orientation = FieldOrientation.getOrientation();
//        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
//
//        dumb.addOption("Yes dumb", true);
//        dumb.addOption("No dumb", false);
//        dumb.setDefaultOption("No dumb", false);
//
//        autoTab.add("Dumb?", dumb)
//                .withWidget(BuiltInWidgets.kComboBoxChooser)
//                .withSize(2, 2)
//                .withPosition(0, 3);
//
//        autoStationPosition.addOption("LA", orientation::getCoralStationLA);
//        autoStationPosition.addOption("LB", orientation::getCoralStationLB);
//        autoStationPosition.addOption("LC", orientation::getCoralStationLC);
//        autoStationPosition.addOption("RA", orientation::getCoralStationRA);
//        autoStationPosition.addOption("RB", orientation::getCoralStationRB);
//        autoStationPosition.addOption("RC", orientation::getCoralStationRC);
//        autoStationPosition.setDefaultOption("RB", orientation::getCoralStationRB);
//
//        autoTab.add("Coral Station Pos", autoStationPosition)
//                .withWidget(BuiltInWidgets.kComboBoxChooser)
//                .withSize(2, 2)
//                .withPosition(2, 3);
//
//        pathCoral.addOption("Yes", true);
//        pathCoral.addOption("No", false);
//        pathCoral.setDefaultOption("Yes", true);
//
//        autoTab.add("Finish Coral?", pathCoral)
//                .withWidget(BuiltInWidgets.kComboBoxChooser)
//                .withSize(2, 2)
//                .withPosition(4, 3);
//
//        for (int i = 0; i < 4; i++) {
//            SendableChooser<Pose2d> reefLocation = new SendableChooser<>();
//            SendableChooser<ReefPosition.ReefLevel> reefLevel = new SendableChooser<>();
//            reefLocation.addOption("None", new Pose2d());
//            reefLocation.addOption("A", orientation.getReefA());
//            reefLocation.addOption("B", orientation.getReefB());
//            reefLocation.addOption("C", orientation.getReefC());
//            reefLocation.addOption("D", orientation.getReefD());
//            reefLocation.addOption("E", orientation.getReefE());
//            reefLocation.addOption("F", orientation.getReefF());
//            reefLocation.addOption("G", orientation.getReefG());
//            reefLocation.addOption("H", orientation.getReefH());
//            reefLocation.addOption("I", orientation.getReefI());
//            reefLocation.addOption("J", orientation.getReefJ());
//            reefLocation.addOption("K", orientation.getReefK());
//            reefLocation.addOption("L", orientation.getReefL());
//            reefLocation.setDefaultOption("None", new Pose2d());
//            reefLevel.addOption("None", ReefPosition.ReefLevel.NONE);
//            reefLevel.addOption("L1", ReefPosition.ReefLevel.L1);
//            reefLevel.addOption("L2", ReefPosition.ReefLevel.L2);
//            reefLevel.addOption("L3", ReefPosition.ReefLevel.L3);
//            reefLevel.addOption("L4", ReefPosition.ReefLevel.L4);
//            reefLevel.setDefaultOption("None", ReefPosition.ReefLevel.NONE);
//
//            autoTab.add(("Position " + (i + 1)), reefLocation)
//                .withWidget(BuiltInWidgets.kComboBoxChooser)
//                .withSize(1, 1)
//                .withPosition(i, 0);
//            autoTab.add(("Level " + (i + 1)), reefLevel)
//                    .withWidget(BuiltInWidgets.kComboBoxChooser)
//                    .withSize(1, 1)
//                    .withPosition(i, 1);
//
//            reefLocations.add(reefLocation);
//            reefLevels.add(reefLevel);
//        }
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
    }

//    public Command getAutoCommand(){
//        if (dumb.getSelected()){
//            return new Dumb(swerveSubsystem);
//        }
//        List<ReefPosition> reefPositions = new ArrayList<>();
//        for (int i = 0; i < 4; i++) {
//            Pose2d reefLocation = reefLocations.get(i).getSelected();
//            ReefPosition.ReefLevel reefLevel = reefLevels.get(i).getSelected();
//            if (!reefLocation.equals(new Pose2d()) && !reefLevel.equals(ReefPosition.ReefLevel.NONE)){
//                reefPositions.add(new ReefPosition(reefLocation, reefLevel));
//            }
//        }
//
//        return new AutoCommandGroup(elevatorSubsystem, coralizerSubsystem, swerveSubsystem, autoStationPosition.getSelected(), reefPositions, pathCoral.getSelected());
//    }

}
