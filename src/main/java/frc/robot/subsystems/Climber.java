// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static Climber instance;

  private final WPI_TalonSRX leftClimber, rightClimber;
  private static DigitalInput rightHallEffect, leftHallEffect;
  private Encoder leftEncoder, rightEncoder;
  //private Encoder leftEncoder = new Encoder();

  private Climber() {
    rightHallEffect = new DigitalInput(Constants.Climber.rightHallEffectPort);
    leftHallEffect = new DigitalInput(Constants.Climber.leftHallEffectPort);
    
    leftClimber = new WPI_TalonSRX(Constants.Climber.leftClimberMotorPort);
    leftClimber.setInverted(true);
    leftClimber.setNeutralMode(NeutralMode.Brake);

    rightClimber = new WPI_TalonSRX(Constants.Climber.rightClimberMotorPort);
    rightClimber.setInverted(true);
    rightClimber.setNeutralMode(NeutralMode.Brake);

    // leftEncoder = new Encoder(new DigitalInput(Constants.Climber.leftChannelA), new DigitalOutput(Constants.Climber.leftChannelB));
    // rightEncoder = new Encoder(new DigitalInput(Constants.Climber.rightChannelA), new DigitalOutput(Constants.Climber.rightChannelB));
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
    // double height = Constants.Climber.encoderConversionRateToCm * leftEncoder.getRaw();

    // if (getLeftHallEffectValue() == true && Math.abs(height) < Constants.Climber.encoderErrorRange) {
    //   height = 0;
    // }
    return 0;
  }

  public double getRightHeight() {
    // double height = Constants.Climber.encoderConversionRateToCm * rightEncoder.getRaw();
    // if (getRightHallEffectValue() == true && Math.abs(height) < Constants.Climber.encoderErrorRange) {
    //   height = 0;
    // }
    return 0;
  }

  public double getAverageHeight()  {
    return (getLeftHeight() + getRightHeight()) / 2.0;
  }

  public boolean getLeftHallEffectValue() {
    return !leftHallEffect.get();
  }

  public boolean getRightHallEffectValue() {
    return !rightHallEffect.get();
  }

  public void setLeftClimberSpeed(double climbSpeed) {
    if (getLeftHallEffectValue() == true && climbSpeed < 0) {
      climbSpeed = 0;
    }
    leftClimber.set(climbSpeed);
  }

  public void setRightClimberSpeed(double climbSpeed) {
    if (getRightHallEffectValue() == true && climbSpeed < 0) {
      climbSpeed = 0;
    }

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

  public boolean climberAtMax() {
    return Math.abs(Constants.Climber.maxClimberHeight - getAverageHeight()) < Constants.Climber.encoderErrorRange;
  }

  public boolean climberAtBottom() {
    return Math.abs(0 - getAverageHeight()) < Constants.Climber.encoderErrorRange;
  }

  public boolean climberAtMiddleRung() {
    return Math.abs(Constants.Climber.middleBarHeight - getAverageHeight()) < Constants.Climber.encoderErrorRange;
  }
 }

