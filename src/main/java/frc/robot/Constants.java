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
    public static final double TICKS_PER_METER = 26199.13932126; // * (1.339280 / 2.13);
    public static final double TICKS_PER_INCH = 76.1120944;
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

        public static double driveP = 0.3, driveI = 0, driveD = 0;
        public static double distanceToleranceMeters = 0.02;
        public static double velocityToleranceMetersPerSec = 0.35;
        public static double pidMotorDeadzone = 0.3;
        public static double pidMinMotorVal = 0.35;
    }
}
