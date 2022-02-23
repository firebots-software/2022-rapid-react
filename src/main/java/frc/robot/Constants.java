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

    public static final class Climber {
        public final static int rightHallEffectPort = 15; //todo
        public final static int leftHallEffectPort = 16; //todo
        public final static int leftClimberMotorPort = 17; //todo
        public final static int rightClimberMotorPort = 15; //todo
        public final static double globalClimbSpeed = 0.65;
        public final static double lowBarHeight = 50; //todo: find height in centimeters
        public final static double middleBarHeight = 100; //todo: find height in centimeters
        public final static double maxClimberHeight = 400; //todo find this height
        public final static double retractedHeight = 20;
        public final static double encoderErrorRange = 5; //ensure this is a good range
        public final static double encoderConversionRateToCm = 1; //todo: find encoder conversion rate
          //Both of these are random numbers replace when we actually now the correct port
    }
}
