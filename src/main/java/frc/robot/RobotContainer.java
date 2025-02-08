// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

import static frc.robot.Constants.cXbox;

public class RobotContainer
{
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
