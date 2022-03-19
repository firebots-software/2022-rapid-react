// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Drivetrain drivetrain;
  private Turret turret;
  private Intake intake;

  private Limelight limelight;
  private RobotContainer m_robotContainer;

  private Shooter shooter;
  private Climber climber;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // init cameraServer + stream
    m_robotContainer = new RobotContainer();
    limelight = Limelight.getInstance();
    turret = Turret.getInstance();
    drivetrain = Drivetrain.getInstance();
    intake = Intake.getInstance();

    // CameraServer is responsible for publishing about cameras/camera servers to
    // Network Tables

    // startAutomaticCapture: creates server for viewing camera feed from dashboard

    try {
      // UsbCamera shooterCamera = CameraServer.startAutomaticCapture("Shooter
      // Camera", 0);
      // UsbCamera intakeCamera = CameraServer.startAutomaticCapture("Intake Camera",
      // 1);

      // shooterCamera.setResolution(640, 480);
      // intakeCamera.setResolution(640, 480);

      new Thread(() -> {
        UsbCamera frontCamera = new UsbCamera("front cam", 0);
        // UsbCamera backCamera = new UsbCamera("back cam", 1);
        frontCamera.setFPS(30);
        // backCamera.setFPS(30);
        frontCamera.setBrightness(10);
        // backCamera.setBrightness(10);

        // frontCamera = CameraServer.startAutomaticCapture("Front Camera", 0);

        // // BACK Orientation
        // backCamera = CameraServer.startAutomaticCapture("Back Camera", 1);

        frontCamera.setResolution(160, 120);
        // backCamera.setResolution(160,120);

        CvSink cvSink1 = new CvSink("front cam sink");
        cvSink1.setSource(frontCamera);
        // CvSink cvSink2 = new CvSink("back cam sink");
        // cvSink2.setSource(backCamera);
        // Put video Blur -> stream on Shuffleboard
        CvSource outputStream = CameraServer.putVideo("Camera Output", 160, 120);

        Mat source = new Mat();
        Mat output = new Mat();

        while (!Thread.interrupted()) {
          if (drivetrain.isUsingFrontCam()) {
            if (cvSink1.grabFrame(source) == 0) {
              continue;
            }
          }
          // else{
          // if (cvSink2.grabFrame(source) == 0) {
          // continue;
          // }
          // }

          // Image processing goes here
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGRA2BGR);
          outputStream.putFrame(output);
        }

        cvSink1.close();
        // cvSink2.close();
      }).start();
      Shuffleboard.update();
    } catch (Exception e) {
      System.err.println("Error initializing camera");
    }
    shooter = Shooter.getInstance();
    shooter.stopBothMotors();
    shooter = Shooter.getInstance();
    climber = Climber.getInstance();

    drivetrain.setMotorNeutralMode(NeutralMode.Brake);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("tx", limelight.getTx());
    SmartDashboard.putNumber("ty", limelight.getTy());
    SmartDashboard.putNumber("drivetrain angular velocity", drivetrain.getAngularVelocity()); 

    // // SmartDashboard.putNumber("name", subsystem.getNumberValue());
    // SmartDashboard.putBoolean("isSlowModeActivated", drivetrain.getSlowModeStatus());
    // SmartDashboard.putString("driveOrientationName", drivetrain.getDriveOrientation().name());

    SmartDashboard.putNumber("top shooter rpm", shooter.getTopShooterRPM());
    SmartDashboard.putNumber("bottom shooter rpm", shooter.getBottomShooterRPM());

    SmartDashboard.putBoolean("adjustable rpm?", shooter.isRPMAdjusting());
    SmartDashboard.putNumber("limelight distance", limelight.getDistanceToTarget());
    SmartDashboard.putNumber("limelight average distance", limelight.getAverageDistance()); 

    SmartDashboard.putNumber("turret encoder degrees", turret.getEncoderValDegrees());
    SmartDashboard.putNumber("turret degrees per second", turret.getDegreesPerSec()); 
    SmartDashboard.putNumber("drivetrain degrees per second", drivetrain.getAngularVelocity()); 
    SmartDashboard.putNumber("real drivetrain deg per sec?", drivetrain.getGyroArray()[2]); 

    // SmartDashboard.putNumber("Right encoder count meters", drivetrain.getRightEncoderCountMeters());
    // SmartDashboard.putNumber("Right encoder velocity", drivetrain.getRightEncoderVelocityMetersPerSec());

    // SmartDashboard.putNumber("Left encoder count meters", drivetrain.getLeftEncoderCountMeters());
    // SmartDashboard.putNumber("Left encoder velocity", drivetrain.getLeftEncoderVelocityMetersPerSec());
    // SmartDashboard.putNumber("Gyro Value", drivetrain.getHeading());

    // SmartDashboard.putBoolean("isCurvatureModeOn", drivetrain.getDriveStatus());

    SmartDashboard.putBoolean("within shooting range?", limelight.isWithinShootingRange());

    // SmartDashboard.putNumber("left drivetrain voltage", drivetrain.getLeftVoltage());
    // SmartDashboard.putNumber("right drivetrain voltage", drivetrain.getRightVoltage());

    // SmartDashboard.putBoolean("left climber hall effect", climber.getLeftHallEffectValue());
    // SmartDashboard.putBoolean("right climber hall effect", climber.getRightHallEffectValue());

    // SmartDashboard.putBoolean("intake piston status", intake.pistonExtended());

    SmartDashboard.putNumber("TESTING: TARGET RPM TOP", shooter.getTopTargetRPM());
    SmartDashboard.putNumber("TESTING: TARGET RPM BOTTOM", shooter.getBottomTargetRPM());

    SmartDashboard.putBoolean("SHOOTER AT RPM", shooter.atTargetRPM());

    // SmartDashboard.putNumber("turret degrees per sec", turret.getDegreesPerSec());

    // SmartDashboard.putNumber("turret motion magic position",
    // turret.getMotionMagicPosition());

    SmartDashboard.putNumber("top shooter error", shooter.getTopTargetRPM() - shooter.getTopShooterRPM());
    SmartDashboard.putNumber("bottom shooter error", shooter.getBottomTargetRPM() - shooter.getBottomShooterRPM());

    SmartDashboard.putBoolean("TURRET AIMED?", limelight.isAimed());

    SmartDashboard.putNumber("last known tx", limelight.getLastKnownTx());
    SmartDashboard.putBoolean("limelight tv", limelight.getTv());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    updateShuffleboard();
    drivetrain.setMotorNeutralMode(NeutralMode.Coast);
    limelight.setLedStatus(false);
    // intake.retractIntake();
  }

  @Override
  public void disabledPeriodic() {
    updateShuffleboard();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    drivetrain.resetOdometry(new Pose2d());
    drivetrain.setMotorNeutralMode(NeutralMode.Brake);
    // turret.zeroEncoder();
    limelight.setLedStatus(true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    updateShuffleboard();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // turret.zeroEncoder();
    limelight.setLedStatus(true);
    drivetrain.setMotorNeutralMode(NeutralMode.Brake);
    drivetrain.resetEncoders();
    // intake.extendIntake();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    updateShuffleboard();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
