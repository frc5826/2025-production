// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

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

    public static final Joystick cJoystick= new Joystick(0);
    
    public static class Elevator {
        public static final int cElevatorMotor1ID = 1;
        public static final int cElevatorMotor2ID = 2;

        //TODO Real values
        public static final int cElevatorEncoder1IDA = 1;
        public static final int cElevatorEncoder1IDB = 0;
        public static final int cElevatorEncoder2IDA = -1;
        public static final int cElevatorEncoder2IDB = -1;

        public static final double cElevatorP = 1;
        public static final double cElevatorI = 0;
        public static final double cElevatorD = 0;
        public static final double cElevatorMinOutput = -0.2;
        public static final double cElevatorMaxOutput = 0.2;
        public static final double cElevatorV = 1;
        public static final double cElevatorG = 1;
        public static final double cElevatorMaxAcceleration = 1;
        public static final double cElevatorMaxVelocity = 1;
        public static final double cElevatorHeightMin = 0;//TODO
        public static final double cElevatorHeightMax = 2;//TODO
        public static final double cElevatorClicksPerMeter = 17777;//TODO find actual value
        public static final double cElevatorRPMtoMPS = 0.9753;//TODO find actual value
        public static final double cElevatorDeadband = 0.01;

    }
}
