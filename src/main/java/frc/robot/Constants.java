// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ButtonBoard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

    //public static final PowerDistribution cPowerDistribution = new PowerDistribution();

    public static class Elevator {
        public static final int cElevatorMotor1ID = 1;
        public static final int cElevatorMotor2ID = 2;

        public static final int cElevatorEncoder1IDA = 1;
        public static final int cElevatorEncoder1IDB = 0;

        public static final double cElevatorP = 2;
        public static final double cElevatorI = 0;
        public static final double cElevatorD = 0.01;
        public static final double cElevatorMinOutput = -1;
        public static final double cElevatorMaxOutput = 1;
        public static final double cElevatorVUp = 0.8;
        public static final double cElevatorVDown = 0.8; //TODO THESE ARE NAMED INCORRECTLY WYATT!!!!!!!!!!!!!!!!!!
        public static final double cElevatorG = 0.07;
        public static final double cElevatorMaxAcceleration = 1;
        public static final double cElevatorMaxVelocity = 1.67;
        public static final double cElevatorHeightMin = 0;
        public static final double cElevatorHeightMax = 1.71;
        public static final double cElevatorDeadband = 0.06;
        //TODO get limit switch IDs
        public static final int cElevatorLimitSwitch1ID = 0;
        public static final int cElevatorLimitSwitch2ID = 0;

        public static class ConversionFactor {
            public static final double cElevatorClicksPerMeter = 17777;
            public static final double cElevatorOverlapPosition = 0.877;
            public static final double cElevatorOverlapConversion = 1.18;
            public static final double cElevatorGearboxRatio = 0.2;
            public static final double cElevatorPulleyRatio = 1.5;
            public static final double cElevatorPulleyRadius = 1.375 * 0.0254 * Math.PI;
            public static final double cElevatorRPMtoMPS = cElevatorGearboxRatio * cElevatorPulleyRatio * cElevatorPulleyRadius * (1.0 / 60);
        }
    }

    public static final ButtonBoard cButtonBoard = new ButtonBoard(25);
    public static final XboxController cXbox = new XboxController(1);

    public static final double cTeleDriveDeadband = 0.25;
    public static final double cTeleTurnDeadband = 0.25;
    public static final double cTurnDeadband = 0.8;

    public static final double cL4offset = 0.6;

    public static class  Swerve {
        public static final double cMaxVelocity = 5.05968; //DON'T CHANGE!!!!!!!!!!!!!!!!!!!!!!!
        public static final double cMaxAngularVelocity = 5;

        public static final PIDConstants cDrivePID = new PIDConstants(0.8, 0, 0);
        public static final PIDConstants cTurnPID = new PIDConstants(4, 0, 0);

        public static final double cHighSpeedMultiplier = 1;
        public static final double cLowSpeedMultiplier = 0.5;
        public static final double cHighCrabSpeedMult = 1;
        public static final double cLowCrabSpeedMult = 0.5;
    }

    public static final double cLOFRejectionValue = 0.3;
    public static final int cLOFk = 3;
    public static final int cLOFTagLimit = 20;

    public static class BluePositions {
        public static Pose2d reefA = new Pose2d(3.16, 4.19, Rotation2d.fromDegrees(0));
        public static Pose2d reefB = new Pose2d(3.16, 3.86, Rotation2d.fromDegrees(0));
        public static Pose2d reefC = new Pose2d(3.68, 2.96, Rotation2d.fromDegrees(60));
        public static Pose2d reefD = new Pose2d(3.97, 2.80, Rotation2d.fromDegrees(60));
        public static Pose2d reefE = new Pose2d(5.00, 2.80, Rotation2d.fromDegrees(120));
        public static Pose2d reefF = new Pose2d(5.30, 2.96, Rotation2d.fromDegrees(120));
        public static Pose2d reefG = new Pose2d(5.82, 3.86, Rotation2d.fromDegrees(180));
        public static Pose2d reefH = new Pose2d(5.82, 4.19, Rotation2d.fromDegrees(180));
        public static Pose2d reefI = new Pose2d(5.30, 5.09, Rotation2d.fromDegrees(-120));
        public static Pose2d reefJ = new Pose2d(5.00, 5.25, Rotation2d.fromDegrees(-120));
        public static Pose2d reefK = new Pose2d(3.97, 5.25, Rotation2d.fromDegrees(-60));
        public static Pose2d reefL = new Pose2d(3.68, 5.09, Rotation2d.fromDegrees(-60));
        public static Pose2d processor = new Pose2d(11.51, 7.47, Rotation2d.fromDegrees(90));
        public static Pose2d coralStationLA = new Pose2d(0.76, 6.69, Rotation2d.fromDegrees(-144));
        public static Pose2d coralStationLB = new Pose2d(1.19, 7, Rotation2d.fromDegrees(-144));
        public static Pose2d coralStationLC = new Pose2d(1.63, 7.32, Rotation2d.fromDegrees(-144));
        public static Pose2d coralStationRA = new Pose2d(0.76, 1.34, Rotation2d.fromDegrees(-36));
        public static Pose2d coralStationRB = new Pose2d(1.19, 1.02, Rotation2d.fromDegrees(-36));
        public static Pose2d coralStationRC = new Pose2d(1.63, 0.71, Rotation2d.fromDegrees(-36));
    }

    public static class Coralizer{
        public static final double cCoralizerP = 0.025;
        public static final double cCoralizerI = 0;
        public static final double cCoralizerD = 0;
        public static final double cCoralizerG = 0.04;
        public static final double cCoralizerMin = -0.75;
        public static final double cCoralizerMax = 0.75;
        public static final double cCoralizerDeadband = 1.25;

        public static final double cCoralizerMinRotation = -90;
        public static final double cCoralizerMaxRotation = 90;
    }

}
