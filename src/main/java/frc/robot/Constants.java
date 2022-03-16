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
        public final static int leftFollowerPort = 4; //TODO: change port numbers for new drivetrain
        public final static int leftMasterPort = 3;
        public final static int rightMasterPort = 2;
        public final static int rightFollowerPort = 1;

        public static final double TICKS_PER_METER = 26199.13932126; // * (1.339280 / 2.13);
        public static final double drivetrainTicksPerRev = 2048;
        public static final double drivetrainWheelRadiusMeters = 0.0762; // convert to meters 

        public final static int PIGEON_ID = 8;

        public static double driveP = 0.3, driveI = 0, driveD = 0;
        public static double angleP = 0.007, angleI = 0, angleD = 0.001;
        public static double distanceToleranceMeters = 0.02;
        public static double angleToleranceDegrees = 15;
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

    public static final class Limelight {
        public static final double alignP = 0.1; 
        public static final double alignI = 0.0; 
        public static final double alignD = 0.0071; 

        public final static double turnToleranceDeg = 5;
        public static final double turnRateToleranceDegPerS = 45; // TODO: understand why this is 45 (might be wrong)

        public static final double limelightAngleOffset = 28; // TODO: change limelight angle offset to match actual thing\
        public static final double heightOfTarget = 104; // in meters 
        public static final double FEED_FORWARD_KS = 0;
        public static final double FEED_FORWARD_KV = 0;
        public static final double FEED_FORWARD_KA = 0;

        public static final double LIMELIGHT_DEFAULT_VALUE = 150.935; 
        // public static final double turretSpeedConstant = 0.1; 
        public static double maxAngle = 90;

        public static double maxDistance = 121;
        public static double minDistance = 90;
    }   

    public static final class Shooter{
        public final static int rollerMotorPort = 12; //TODO: change to actual port
        public final static int shooterTopMotorPort = 7; //TODO: change port number
        public final static int shooterBottomMotorPort = 6; //TODO: change port number
        public static final double shooterEncoderTicksPerRev = 2048;
        public static final double motorSpeedToleranceRPM = 5.0; //todo: change to real tolerance

        public static final double DEFAULT_SHOOTER_SPEED = 0.5;
        public static final double FIXED_RPM = 4000;
        public static final double MAX_RPM = 6000;
        public static final double MAX_VOLTAGE = 12.0;
        public static final double SHOOTER_TARGET_SPEED = 0.7; 
        public static final double maxRollerSpeed = 0.3; //TODO: change to actual max

        // CONTROLLERS
        public static final double kpFlywheel = 0.14172 / (MAX_VOLTAGE * 60); // divide by max voltage * 60 s/min

        public static final double ksTopFlywheel = 0.4754;
        public static final double kvTopFlywheel = 0.1062;
        public static final double kaTopFlywheel = 0.0145;

        public static final double ksBottomFlywheel = 0.51868;
        public static final double kvBottomFlywheel = 0.10913;
        public static final double kaBottomFlywheel = 0.01227;

    
    }
    public static final class Turret{
        public final static int motorPortNumber = 5; //TODO: change to actual ID 
        public static final double turretGearRatio = 10.0 * 14.0; //(small gear ration * big gear ratio )
        public static final double encoderTicksPerRev = 2048 * turretGearRatio;
        public static final double encoderTicksPerDegree = encoderTicksPerRev / 360.0;
        public static final double pidPositionToleranceDegrees = 1;
        public static final double pidVelToleranceDegPerSecond = 0.5;

        public static final double ksTurret = 0.55865;
        public static final double kvTurret = 0.041588;
        public static final double kaTurret = 0.00096306;
        public static final double constantTurretTurnSpeed = 0.5; 
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
        public final static int rightHallEffectPort = 2; //todo
        public final static int leftHallEffectPort = 3; //todo
        public final static int leftClimberMotorPort = 10; //todo
        public final static int rightClimberMotorPort = 11; //todo
        public final static double climbSpeedUp = 0.7;
        public final static double climbSpeedDown = -1;
        public final static double lowBarHeight = 50; //todo: find height in centimeters
        public final static double middleBarHeight = 100; //todo: find height in centimeters
        public final static double maxClimberHeight = 400; //todo find this height
        public final static double retractedHeight = 20;
        public final static double encoderErrorRange = 5; //ensure this is a good range
        public final static double encoderConversionRateToCm = 1; //todo: find encoder conversion rate
          //Both of these are random numbers replace when we actually now the correct port
        public final static int leftChannelA = 8;
        public final static int leftChannelB = 9;
        public final static int rightChannelA = 6;
        public final static int rightChannelB = 7;
    }

    public static final class Intake {
        public final static int PISTON_PORT = 2;
        public final static int RIGHT_PISTON_PORT = 3 ;
        public final static int INTAKE_MOTOR_PORT = 9;
        public final static int SPAGHETTI_MOTOR_PORT = 8;

        public static final double INTAKE_SPEED_FORWARDS = -1;
        public static final double INTAKE_SPEED_BACKWARDS = 1;

        public static final double SPAGHETTI_SPEED = 0.7;
    }
}
