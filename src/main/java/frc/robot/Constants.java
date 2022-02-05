// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OI {
        public static final int PS4_CONTROLLER_PORT = 3;

        // Buttons on PS4 Controller
        public static final int SQUARE_BUTTON_PORT = 1;
        public static final int X_BUTTON_PORT = 2;
        public static final int CIRCLE_BUTTON_PORT = 3;
        public static final int TRIANGLE_BUTTON_PORT = 4;
        public static final int L1_BUTTON_PORT = 5;
        public static final int R1_BUTTON_PORT = 6;
        public static final int L2_BUTTON_PORT = 7;
        public static final int R2_BUTTON_PORT = 8;
        public static final int PS_SHARE_BUTTON_PORT = 9;
        public static final int OPTIONS_BUTTON_PORT = 10;
        public static final int L3_BUTTON_PORT = 11;
        public static final int R3_BUTTON_PORT = 12;
        public static final int PS_BUTTON_PORT = 13;
        public static final int BIG_BUTTON_PORT = 14;
    }

    public static final class Drivetrain {
        public final static int leftFollowerPort = 1; //TODO: change port numbers for new drivetrain
        public final static int leftMasterPort = 2;
        public final static int rightMasterPort = 4;
        public final static int rightFollowerPort = 3;

        public static final double TICKS_PER_METER = 26199.13932126; // * (1.339280 / 2.13);

        public static double driveP = 0.3, driveI = 0, driveD = 0;
        public static double distanceToleranceMeters = 0.02;
        public static double velocityToleranceMetersPerSec = 0.35;
        public static double pidMotorDeadzone = 0.3;
        public static double pidMinMotorVal = 0.35;
    }

    public static final class Limelight {
        public static final double alignP = 0.2; 
        public static final double alignI = 0.0; 
        public static final double alignD = 0.0071; 

        public final static double turnToleranceDeg = 3;
        public static final double turnRateToleranceDegPerS = 45; // TODO: understand why this is 45 (might be wrong)

        public static final double limelightAngleOffset = 74; // TODO: change limelight angle offset to match actual thing\
        public static final double heightOfTarget = 2.64; // in meters 
        // public static final double turretSpeedConstant = 0.1; 
    }   

    public static final class Shooter{
        public final static int shooterPistonPort = 2; //TODO: change to actual port 
        public final static int shooterMotorPort = 1; //TODO: change port number
        public static final double shooterEncoderTicksPerRev = 2048; //todo: change lol
        public static final double motorSpeedToleranceRPM = 5.0; //todo: change to real tolerance

        public static final double DEFAULT_SHOOTER_SPEED = 0.5;
    
    }
    public static final class Turret{
        public final static int motorPortNumber = 5; //TODO: change to actual ID 
        public static final double turretGearRatio = 45.0 * 14.0; //(small gear ration * big gear ratio )
        public static final double encoderTicksPerRev = 2048 * turretGearRatio;
        public static final double encoderTicksPerDegree = encoderTicksPerRev / 360.0;
        public static final double pidPositionToleranceDegrees = 2;
        public static final double pidVelToleranceDegPerSecond = 10;
    }
}
