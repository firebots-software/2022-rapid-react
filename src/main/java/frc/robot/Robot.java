// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.driveOrientation;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static Drivetrain driveTrain;
  private RobotContainer m_robotContainer;
  private Drivetrain drivetrain;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //init cameraServer + stream 
    m_robotContainer = new RobotContainer();
    driveTrain = Drivetrain.getInstance();

    // CameraServer is responsible for publishing about cameras/camera servers to Network Tables

    // startAutomaticCapture: creates server for viewing camera feed from dashboard 

    try {
      // UsbCamera shooterCamera = CameraServer.startAutomaticCapture("Shooter Camera", 0);
      // UsbCamera intakeCamera = CameraServer.startAutomaticCapture("Intake Camera", 1);
      
      // shooterCamera.setResolution(640, 480);
      // intakeCamera.setResolution(640, 480);

      new Thread(() -> {
        UsbCamera frontCamera, backCamera;

        
        frontCamera = CameraServer.startAutomaticCapture("Camera", 0);
      
          // BACK Orientation
        backCamera = CameraServer.startAutomaticCapture("Camera", 1);

        frontCamera.setResolution(640, 480);
        backCamera.setResolution(640,480);
  
        CvSink cvSink1 = CameraServer.getVideo(frontCamera);
        CvSink cvSink2 = CameraServer.getVideo(backCamera);
        // Put video Blur -> stream on Shuffleboard
        CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
  
        Mat frontSource = new Mat();
        Mat frontOutput = new Mat();
        Mat backSource = new Mat();
        Mat backOutput = new Mat();
        
        
        while(!Thread.interrupted()) {
          if(driveTrain.getOrientation() == Drivetrain.driveOrientation.FRONT){
            if (cvSink1.grabFrame(frontSource) == 0) {
              continue;
            }
            // Image processing goes here
            Imgproc.cvtColor(frontSource, frontOutput, Imgproc.COLOR_BGR2GRAY);
            outputStream.putFrame(frontOutput);
          }
          if(driveTrain.getOrientation() == Drivetrain.driveOrientation.BACK){
            if (cvSink2.grabFrame(backSource) == 0) {
              continue;
            }
            // Image processing goes here
            Imgproc.cvtColor(backSource, backOutput, Imgproc.COLOR_BGR2GRAY);
            outputStream.putFrame(backOutput);
          }
        }
      }).start();
      Shuffleboard.update();
    } catch(Exception e){
      System.err.println("Error initializing camera");
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  private void updateShuffleboard() {
    // SmartDashboard.putNumber("name", subsystem.getNumberValue());
    SmartDashboard.putBoolean("isSlowModeActivated", drivetrain.getSlowModeStatus());
    SmartDashboard.putString("driveOrientationName", drivetrain.getDriveOrientation().name());
  }


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    updateShuffleboard();
  }

  @Override
  public void disabledPeriodic() {
    updateShuffleboard();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
  public void testPeriodic() {}
}
