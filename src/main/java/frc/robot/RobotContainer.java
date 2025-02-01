// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.coralizer.CoralizerIntakeCommand;
import frc.robot.commands.coralizer.CoralizerWristCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.elevator.ElevatorRepositionCommand;
import frc.robot.subsystems.CoralizerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.cJoystick;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public final ElevatorPositionCommand elevatorHomeCommand = new ElevatorPositionCommand(elevatorSubsystem, 0);

    public final ElevatorRepositionCommand elevatorRepositionCommandDown = new ElevatorRepositionCommand(elevatorSubsystem, -0.1);
    public final ElevatorRepositionCommand elevatorRepositionCommandUp = new ElevatorRepositionCommand(elevatorSubsystem, 0.1);
    public final ElevatorRepositionCommand elevatorRepositionCommandBigDown = new ElevatorRepositionCommand(elevatorSubsystem, -0.5);
    public final ElevatorRepositionCommand elevatorRepositionCommandBigUp = new ElevatorRepositionCommand(elevatorSubsystem, 0.5);

    public final CoralizerSubsystem coralizerSubsystem = new CoralizerSubsystem();

    public final CoralizerIntakeCommand coralizerInCommand = new CoralizerIntakeCommand(coralizerSubsystem, 0.75);
    public final CoralizerIntakeCommand coralizerOutCommand = new CoralizerIntakeCommand(coralizerSubsystem, -1);
    public final CoralizerWristCommand coralizerWristCommandUp = new CoralizerWristCommand(coralizerSubsystem, 45);
    public final CoralizerWristCommand coralizerWristCommandDown = new CoralizerWristCommand(coralizerSubsystem, 0);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
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
        new Trigger(cJoystick::getTrigger).whileTrue(coralizerInCommand);

        new Trigger(() -> cJoystick.getRawButton(2)).whileTrue(coralizerOutCommand);

        new Trigger(() -> cJoystick.getRawButton(3)).onTrue(coralizerWristCommandDown);
        new Trigger(() -> cJoystick.getRawButton(5)).onTrue(coralizerWristCommandUp);
//        new Trigger(() -> cJoystick.getRawButton(4)).onTrue(elevatorRepositionCommandBigDown);
//        new Trigger(() -> cJoystick.getRawButton(6)).onTrue(elevatorRepositionCommandBigUp);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return null;
    }
}
