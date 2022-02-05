// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static Climber instance;

  private final WPI_TalonSRX leftClimber, rightClimber;
  private static DigitalInput rightHallEffect;
  private static DigitalInput leftHallEffect;

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
  
  public double getLeftEncoderValue() {
    return leftClimber.getSelectedSensorPosition();
  }

  public double getRightEncoderValue() {
    return rightClimber.getSelectedSensorPosition();
  }

  public double getAverageEncoderValues()  {
    return (getLeftEncoderValue() + getRightEncoderValue())/2;
  }

  public boolean getLeftHallEffectValue() {
    return leftHallEffect.get();
  }

  public boolean getRightHallEffectValue() {
    return rightHallEffect.get();
  }
  
  public void climbToHangar(double climbSpeed, double height) {
    double leftEncoderVal = getLeftEncoderValue();
    double rightEncoderVal = getRightEncoderValue();

    boolean leftHallEffectVal = getLeftHallEffectValue();
    boolean rightHallEffectVal = getRightHallEffectValue();

    // if (leftEncoderVal < height && climbSpeed > 0) {
    //   leftClimber.set(climbSpeed);
    // } else if (leftEncoderVal > 0 && climbSpeed < 0) {
    //   leftClimber.set(climbSpeed);
    // } else {
    //   leftClimber.stopMotor();
    // }

    // if (rightEncoderVal < height && climbSpeed > 0) {
    //   rightClimber.set(climbSpeed);
    // } else if (leftEncoderVal > 0 && climbSpeed < 0) {
    //   rightClimber.set(climbSpeed);
    // } else {
    //   rightClimber.stopMotor();
    // }

    if (leftEncoderVal < height && climbSpeed > 0 && leftHallEffectVal) {
      leftClimber.set(climbSpeed);
    } else if (leftEncoderVal > 0 && climbSpeed < 0) {
      leftClimber.set(climbSpeed);
    } else {
      leftClimber.stopMotor();
    }

    if (rightEncoderVal < height && climbSpeed > 0 && rightHallEffectVal) {
      rightClimber.set(climbSpeed);
    } else if (leftEncoderVal > 0 && climbSpeed < 0) {
      rightClimber.set(climbSpeed);
    } else {
      rightClimber.stopMotor();
    }
  }

  /***
   * Sets the left and right climber motors to a specific voltage
   * @param climbSpeed Motor voltage values between -1 and 1 inclusive
   */
  public void setClimberSpeed(double climbSpeed) {
    leftClimber.set(climbSpeed);
    rightClimber.set(climbSpeed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
