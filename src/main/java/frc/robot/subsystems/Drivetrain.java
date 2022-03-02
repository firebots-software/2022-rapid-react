// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.CurvatureDrive;
import frc.robot.commands.drivetrain.JoystickDrive;

public class Drivetrain extends SubsystemBase {
  // constants
  private static final double DEADZONE_RANGE = 0.25;
  private final double SLOW_MODE_CONSTANT = 0.3;
  private final double DEFAULT_DRIVE_CONSTANT = 0.7;
  private final double RAMPING_CONSTANT = 1;
  private boolean usingFrontCam = true;


/*
    MOTION PROFILING
     */
    private final DifferentialDriveOdometry odometry;

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH_METERS);
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0); // 2 m/s and 1 rad/s
    // convert chassis speeds to wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;
  
  // fields
  private final WPI_TalonSRX leftFollower, leftFrontMaster, rightRearMaster, rightFollower;
  // private static Pigeon2 pigeon; 

  private final DifferentialDrive robotDrive;
  private boolean isSlowMode;


  public enum driveOrientation {
    FRONT, BACK;

    public driveOrientation toggle() {
      return this == FRONT ? BACK : FRONT;
    } // if front, set to back; if back, set to front B)

  }

  private driveOrientation orientation;

  // robot status
  private boolean brakeMode;


  /**
   * The Singleton instance of this Drivetrain. External classes should
   * use the {@link #getInstance()} method to get the instance.
   */
  private static Drivetrain instance;

  private boolean curvatureDriveOn;
  /** Creates a new Drivetrain. */
  private Drivetrain() {
    curvatureDriveOn = true;

    this.leftFollower = new WPI_TalonSRX(Constants.Drivetrain.leftFollowerPort);
    this.leftFrontMaster = new WPI_TalonSRX(Constants.Drivetrain.leftMasterPort);
    this.rightRearMaster = new WPI_TalonSRX(Constants.Drivetrain.rightMasterPort);
    this.rightFollower = new WPI_TalonSRX(Constants.Drivetrain.rightFollowerPort);
    resetEncoders();

    // pigeon = new Pigeon2(Constants.Drivetrain.PIGEON_ID);
    resetGyro();
    
    MotorControllerGroup leftSide = new MotorControllerGroup(leftFrontMaster, leftFollower);
    MotorControllerGroup rightSide = new MotorControllerGroup(rightFollower, rightRearMaster);
    robotDrive = new DifferentialDrive(leftSide, rightSide);
    configTalons();
    setMotorNeutralMode(NeutralMode.Coast);


    // this.gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    // this.gyro.calibrate();
    // this.gyro.reset();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), new Pose2d());

    this.orientation = driveOrientation.BACK; // default value
    this.isSlowMode = false; // default value

  }

  /**
   * Returns the Singleton instance of this Drivetrain. This static method
   * should be used -- {@code Drivetrain.getInstance();} -- by external
   * classes, rather than the constructor to get the instance of this class.
   */
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderCountMeters(),
                getRightEncoderCountMeters());
  }

  public void arcadeDrive(double frontBackSpeed, double rotation) {
    if (frontBackSpeed < DEADZONE_RANGE && frontBackSpeed > -DEADZONE_RANGE && 
    rotation < DEADZONE_RANGE && rotation > -DEADZONE_RANGE) {
      robotDrive.stopMotor();
    } else {
      if (orientation == driveOrientation.BACK) {
        frontBackSpeed *= -1;
      }
      if (isSlowMode) {
       frontBackSpeed *= SLOW_MODE_CONSTANT;
        rotation *= SLOW_MODE_CONSTANT;

      } else {
        frontBackSpeed *= DEFAULT_DRIVE_CONSTANT;
        rotation *= DEFAULT_DRIVE_CONSTANT;
      }


      frontBackSpeed = restrictToRange(frontBackSpeed, -1, 1);
      rotation = restrictToRange(rotation, -1, 1);

      SmartDashboard.putNumber("frontbackspeed",frontBackSpeed);
      SmartDashboard.putNumber("rotationspeed",rotation);

      robotDrive.arcadeDrive(frontBackSpeed, -rotation);

    }
  }

  public void PIDtankDrive(double leftSpeed, double rightSpeed) {
    rightSpeed = restrictToRange(rightSpeed, -1, 1);
    leftSpeed = restrictToRange(leftSpeed, -1, 1);

    // if(leftSpeed<=Constants.Drivetrain.pidMotorDeadzone){
    //   leftSpeed = Constants.Drivetrain.pidMinMotorVal;
    // }

    // if(rightSpeed<=Constants.Drivetrain.pidMotorDeadzone){
    //   rightSpeed = Constants.Drivetrain.pidMinMotorVal;
    // }

    robotDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void PIDarcadeDrive(double frontBackSpeed){
    frontBackSpeed = restrictToRange(frontBackSpeed, -1, 1);
    robotDrive.arcadeDrive(frontBackSpeed, 0);
  }

  public void PIDarcadeDriveAngle(double angle){
    angle = restrictToRange(angle, -1, 1);
    robotDrive.arcadeDrive(0, angle);
  }


  public void curvatureDrive(double frontBackSpeed, double rotation) {
    boolean quickTurn = false;
    if (frontBackSpeed < DEADZONE_RANGE && frontBackSpeed > -DEADZONE_RANGE && 
    rotation < DEADZONE_RANGE && rotation > -DEADZONE_RANGE) {
      robotDrive.stopMotor();
      
    } else {
      if (orientation == driveOrientation.BACK) {
        frontBackSpeed *= -1;
        
      }
      if (isSlowMode) {
        frontBackSpeed *= SLOW_MODE_CONSTANT;
        rotation *= SLOW_MODE_CONSTANT;

      } else {
        frontBackSpeed *= DEFAULT_DRIVE_CONSTANT;
        rotation *= DEFAULT_DRIVE_CONSTANT;
      }

      if (frontBackSpeed < DEADZONE_RANGE && frontBackSpeed > -DEADZONE_RANGE) {
        quickTurn = true;
        rotation *= 0.7;
        if (!isSlowMode) rotation *= 0.5;
      } else {
        quickTurn = false;
      }

      frontBackSpeed = restrictToRange(frontBackSpeed, -1, 1);
      rotation = restrictToRange(rotation, -1, 1);

      SmartDashboard.putNumber("frontbackspeed",frontBackSpeed);
      SmartDashboard.putNumber("rotationspeed",rotation);
      SmartDashboard.putBoolean("quickTurnEnabled", quickTurn);

      robotDrive.curvatureDrive(frontBackSpeed, rotation, quickTurn);

    }
  }

  public void PIDarcadeDrive(double frontBackSpeed, double rotation) {
    frontBackSpeed = restrictToRange(frontBackSpeed, -1, 1);
    rotation = restrictToRange(rotation, -1, 1);

    robotDrive.arcadeDrive(frontBackSpeed, rotation, false);
}

  public void resetEncoders() {
    leftFrontMaster.setSelectedSensorPosition(0);
    rightRearMaster.setSelectedSensorPosition(0);
  }

  public void configTalons() {
    rightRearMaster.configFactoryDefault();
    leftFrontMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightFollower.configFactoryDefault();


    rightFollower.configOpenloopRamp(RAMPING_CONSTANT);
    rightRearMaster.configOpenloopRamp(RAMPING_CONSTANT);
    leftFollower.configOpenloopRamp(RAMPING_CONSTANT);
    leftFrontMaster.configOpenloopRamp(RAMPING_CONSTANT);

    leftFollower.follow(leftFrontMaster);
    rightFollower.follow(rightRearMaster);

    rightRearMaster.setInverted(false); //might need to change
    leftFrontMaster.setInverted(true); //might need to change
    leftFollower.setInverted(true);
    rightFollower.setInverted(true);

    setMotorNeutralMode(NeutralMode.Coast);
    // robotDrive.setRightSideInverted(true); 
    // THIS IS BROCKEN IN 2022!!!!!! either invert rightSide motorcontrollergroup or invert individual motors
    //robotDrive.setRightSideInverted(false); //dont change for some reason idk why (maybe robot will go backwards idk)
}

public boolean getBrakeModeStatus() {
  return brakeMode;
}

//todo: why is setBrakeMode commented out
public void setBrakeMode(boolean newBrakeMode) {
  //       if (newBrakeMode) {
  //        rightRearMaster.set(NeutralMode.Brake, 0);
  // }
}

public driveOrientation getDriveOrientation() {
  return orientation;
}

public void toggleDriveOrientation() {
  this.orientation = orientation.toggle();
}

public boolean getSlowModeStatus() {
  return isSlowMode;
}

/**
* Sets the drivetrain's slow mode status. If slow mode is on, all velocity values will be reduced.
*
* @param isSlowMode
*/
public void setSlowMode(boolean isSlowMode) {
  this.isSlowMode = isSlowMode;
}

  public void stop() {
    leftFollower.set(0);
    rightRearMaster.set(0);
    leftFrontMaster.set(0);
    rightFollower.set(0);
}

  public void resetGyro() {
    // pigeon.setYaw(0);
  }

  public double getHeading() {
    // if (pigeon != null) {
    //     return pigeon.getYaw(); //todo: why gyro angle = -heading?
    // } else {
    //     return 0;
    // }
    return 0;
  }

  public boolean getDriveStatus() {
    return curvatureDriveOn;
  }

  private double restrictToRange(double n, int min, int max) {
    if (n > max) return max;
    if (n < min) return min;
    return n;
  }
  public double getLeftEncoderCountMeters() {
    return -leftFrontMaster.getSelectedSensorPosition() / Constants.TICKS_PER_METER;
  }

  public double getRightEncoderCountMeters() {
    return -rightRearMaster.getSelectedSensorPosition() / Constants.TICKS_PER_METER;
  }

  public double getRightEncoderTicks() {
    return -rightRearMaster.getSelectedSensorPosition();
  }

  public double getLeftEncoderVelocityMetersPerSec() {
    return (-leftFrontMaster.getSelectedSensorVelocity() * 10) / Constants.TICKS_PER_METER;
  }

  public double getRightEncoderVelocityMetersPerSec() {
    return (-rightRearMaster.getSelectedSensorVelocity() * 10) / Constants.TICKS_PER_METER;
  } 

  public double getAvgEncoderCountMeters(){
    return (getLeftEncoderCountMeters()+getRightEncoderCountMeters())/2;
  }

  public void setMotorNeutralMode(NeutralMode neutralMode){
    rightFollower.setNeutralMode(neutralMode);
    leftFollower.setNeutralMode(neutralMode);
    leftFrontMaster.setNeutralMode(neutralMode);
    rightRearMaster.setNeutralMode(neutralMode);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
}

public void setMaxOutput(double max) {
    robotDrive.setMaxOutput(max);
}

/**
 * Returns the current wheel speeds of the robot.
 *
 * @return The current wheel speeds.
 */
public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(this.getLeftEncoderVelocityMetersPerSec(), this.getRightEncoderVelocityMetersPerSec());
}

/**
 * Resets the odometry to the specified pose.
 *
 * @param pose The pose to which to set the odometry.
 */
public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
}

/**
 * Controls the left and right sides of the drive directly with voltages.
 *
 * @param leftVolts  the commanded left output
 * @param rightVolts the commanded right output
 */
public void tankDriveVolts(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("leftvolts", leftVolts);
    SmartDashboard.putNumber("rightvolts", rightVolts);
    leftFrontMaster.setVoltage(leftVolts);
    leftFollower.setVoltage(leftVolts);
    rightRearMaster.setVoltage(rightVolts);
    rightFollower.setVoltage(rightVolts);


    instance.robotDrive.feed();
}
  public void toggleDefaultCommand(Joystick ps4) {
    if (curvatureDriveOn) {
      this.setDefaultCommand( new JoystickDrive(
        () -> ps4.getRawAxis(1),
        () -> ps4.getRawAxis(2)));

        curvatureDriveOn = false;
    } else {
      this.setDefaultCommand( new CurvatureDrive(
        () -> ps4.getRawAxis(1),
        () -> ps4.getRawAxis(2)));

        curvatureDriveOn = true;
    }
  }


  // Getters
  public driveOrientation getOrientation(){
    return orientation;
  }

  public boolean isUsingFrontCam(){ return usingFrontCam; }

  // Setters
  public void setDriveOrientation(driveOrientation orientation) {
    this.orientation = orientation;
  }

  public void setUsingFrontCam(boolean usingFrontCam){
    this.usingFrontCam = usingFrontCam;
  }

  public double getLeftVoltage() {
    return leftFrontMaster.getBusVoltage();
  }

  public double getRightVoltage() {
    return rightRearMaster.getBusVoltage();
  }
}
