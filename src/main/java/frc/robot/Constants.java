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
        public final static double turnToleranceDeg = 1;
        public static final double turnRateToleranceDegPerS = 45;
        public static double smallTurnP = 0.01, smallTurnI = 0.0, smallTurnD = 0.0036569; //todo: verify if correct values 
    }

    public static final class Shooter{
        public final static int shooterPistonPort = 2; //TODO: change to actual port 
        public final static int rollerMotorPort = 3; //TODO: change to actual port
        public final static int shooterTopMotorPort = 1; //TODO: change port number
        public final static int shooterBottomMotorPort = 1; //TODO: change port number
        public static final double shooterEncoderTicksPerRev = 2048; //todo: change lol
        public static final double motorSpeedToleranceRPM = 5.0; //todo: change to real tolerance

        public static final double DEFAULT_SHOOTER_SPEED = 0.5;
        public static final double SHOOTER_TARGET_SPEED = 0.7; 
        public static final double maxRollerSpeed = 0.5; //TODO: change to actual max
    
    }
    public static final class Turret{
        public final static int motorPortNumber = 2; //TODO: change to actual ID 
      
        public static final double turretGearRatio = 10.0; //TODO: change real number
        public static final double encoderTicksPerRev = 2048 * turretGearRatio;
        public static final double encoderTicksPerDegree = encoderTicksPerRev / 360.0;
        public static final double pidPositionToleranceDegrees = 3.0;
        public static final double pidVelToleranceDegPerSecond = 10;

    
    }
}
