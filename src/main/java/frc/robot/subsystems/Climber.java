// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static Climber instance;

  private final WPI_TalonSRX leftClimber, rightClimber;
  private static DigitalInput rightHallEffect, leftHallEffect;
  private static boolean leftHallEffectVal, rightHallEffectVal;
  //private Encoder leftEncoder = new Encoder();

  private Climber() {
    rightHallEffect = new DigitalInput(Constants.Climber.rightHallEffectPort);
    leftHallEffect = new DigitalInput(Constants.Climber.leftHallEffectPort);
    
    this.leftClimber = new WPI_TalonSRX(Constants.Climber.leftClimberMotorPort);
    this.rightClimber = new WPI_TalonSRX(Constants.Climber.rightClimberMotorPort);
  }

  /**
   * Returns the Singleton instance of this Climber. This static method
   * should be used -- {@code Climber.getInstance();} -- by external
   * classes, rather than the constructor to get the instance of this class.
   */
  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }
  
  public double getLeftHeight() {
    double height = Constants.Climber.encoderConversionRateToCm * leftClimber.getSelectedSensorPosition();

    if (getLeftHallEffectValue() == true && Math.abs(height) < Constants.Climber.encoderErrorRange) {
      height = 0;
    }
    if (getLeftHallEffectValue() == true && Math.abs(Constants.Climber.maxClimberHeight - height) < Constants.Climber.encoderErrorRange) {
      height = (Constants.Climber.maxClimberHeight);
    }

    return height;
  }

  public double getRightHeight() {
    double height = Constants.Climber.encoderConversionRateToCm * rightClimber.getSelectedSensorPosition();

    if (getRightHallEffectValue() == true && Math.abs(height) < Constants.Climber.encoderErrorRange) {
      height = 0;
    }
    if (getRightHallEffectValue() == true && Math.abs(Constants.Climber.maxClimberHeight - height) < Constants.Climber.encoderErrorRange) {
      height = (Constants.Climber.maxClimberHeight);
    }

    return height;
  }

  public double getAverageHeight()  {
    return (getLeftHeight() + getRightHeight()) / 2.0;
  }

  public boolean getLeftHallEffectValue() {
    return leftHallEffect.get();
  }

  public boolean getRightHallEffectValue() {
    return rightHallEffect.get();
  }

  public void setLeftClimberSpeed(double climbSpeed) {
    leftClimber.set(climbSpeed);
  }

  public void setRightClimberSpeed(double climbSpeed) {
    rightClimber.set(climbSpeed);
  }

  /***
   * Sets the left and right climber motors to a specific voltage
   * @param climbSpeed Motor voltage values between -1 and 1 inclusive
   */
  public void setClimberSpeed(double climbSpeed) {
    setLeftClimberSpeed(climbSpeed);
    setRightClimberSpeed(climbSpeed);
  }

  public void stopClimber() {
    setClimberSpeed(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
