// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  // constants
  private static final double DEADZONE_RANGE = 0.1;
  private final double SLOW_MODE_CONSTANT = 0.35;
  private final double RAMPING_CONSTANT = 0.25;


  // fields
  private final WPI_TalonFX leftFollower, leftFrontMaster, rightRearMaster, rightFollower;
  private ADXRS450_Gyro gyro;

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

  /** Creates a new Drivetrain. */
  private Drivetrain() {
    this.leftFollower = new WPI_TalonFX(Constants.Drivetrain.leftFollowerPort);
    this.leftFrontMaster = new WPI_TalonFX(Constants.Drivetrain.leftMasterPort);
    this.rightRearMaster = new WPI_TalonFX(Constants.Drivetrain.rightMasterPort);
    this.rightFollower = new WPI_TalonFX(Constants.Drivetrain.rightFollowerPort);
    resetEncoders();

    MotorControllerGroup leftSide = new MotorControllerGroup(leftFrontMaster, leftFollower);
    MotorControllerGroup rightSide = new MotorControllerGroup(rightFollower, rightRearMaster);
    robotDrive = new DifferentialDrive(leftSide, rightSide);
    configTalons();


    this.gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    this.gyro.calibrate();
    this.gyro.reset();

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
        frontBackSpeed *= 0.5;
        rotation *= 0.5;
      }


      frontBackSpeed = restrictToRange(frontBackSpeed, -1, 1);
      rotation = restrictToRange(rotation, -1, 1);

      SmartDashboard.putNumber("frontbackspeed",frontBackSpeed);
      SmartDashboard.putNumber("rotationspeed",rotation);

      robotDrive.arcadeDrive(frontBackSpeed, rotation);

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

    rightRearMaster.setInverted(true); //might need to change
    leftFrontMaster.setInverted(false); //might need to change
    leftFollower.setInverted(false);
    rightFollower.setInverted(true);

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
    this.gyro.reset();
  }

  public double getHeading() {
    if (gyro != null) {
        return -gyro.getAngle(); //todo: why gyro angle = -heading?
    } else {
        return 0;
    }
}

  private double restrictToRange(double n, int min, int max) {
    if (n > max) return max;
    if (n < min) return min;
    return n;
  }


  // Getters
  public driveOrientation getOrientation(){
    return orientation;
  }

  // Setters
  public void setDriveOrientation(driveOrientation orientation) {
    this.orientation = orientation;
  }
}
