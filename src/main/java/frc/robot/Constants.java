// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double TICKS_PER_METER = 3064; // 26199.13932126; // * (1.339280 / 2.13);
    public static final double TICKS_PER_INCH = (TICKS_PER_METER / 100.0) * 2.54;//76.1120944;
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
        public final static int PIGEON_ID = 8;

        public static double driveP = 0.3, driveI = 0, driveD = 0;
        public static double angleP = 0.0055, angleI = 0, angleD = 0.0001;
        public static double distanceToleranceMeters = 0.02;
        public static double angleToleranceDegrees = 1;
        public static double velocityToleranceMetersPerSec = 0.35;
        public static double velocityToleranceDegreesPerSec = 1;
        public static double pidMotorDeadzone = 0.3;
        public static double pidMinMotorVal = 0.35;

        public static final double TRACK_WIDTH_METERS = 0.612775;

        public static final DifferentialDriveKinematics kinematics =
                new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public static double kMaxSpeedMetersPerSecond = 0.5;
        public static double kMaxAccelerationMetersPerSecondSquared = 0.5;

        // ramsete values from wpilib docs
        public static double kRamseteB = 2.0;
        public static double kRamseteZeta = 0.7;
        public static double kPDriveVel = 5.8082;

        public static final double ksVolts = 1.5451;
        public static final double kvVoltSecondsPerMeter = 3.3875;
        public static final double kaVoltSecondsSquaredPerMeter = 1.2148;

        public static DifferentialDriveVoltageConstraint autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(ksVolts,
                                kvVoltSecondsPerMeter,
                                kaVoltSecondsSquaredPerMeter),
                        kinematics,
                        10);

        public static TrajectoryConfig MotionProfilingConfig = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .setReversed(false);


        public static TrajectoryConfig MotionProfilingConfigReversed = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);
    }
    
    public static class Ramsete {
        public static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts,
                Constants.Drivetrain.kvVoltSecondsPerMeter,
                Constants.Drivetrain.kaVoltSecondsSquaredPerMeter);

        public static RamseteController ramseteController = new RamseteController(Constants.Drivetrain.kRamseteB, Constants.Drivetrain.kRamseteZeta);

        public final static double turnToleranceDeg = 1;
        public static final double turnRateToleranceDegPerS = 45;
        public static double smallTurnP = 0.01, smallTurnI = 0.0, smallTurnD = 0.0036569; //todo: verify if correct values
    }

    public static final class Climber {
        public final static int leftClimberPort = 9999;
        public final static int rightClimberPort = 9998;  //Both of these are random numbers replace when we actually now the correct port
    }
}
