// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static Climber instance;

  private final WPI_TalonSRX leftClimber, rightClimber;
  private static DigitalInput rightHallEffect, leftHallEffect;


  private Climber() {
    rightHallEffect = new DigitalInput(Constants.Climber.rightHallEffectPort);
    leftHallEffect = new DigitalInput(Constants.Climber.leftHallEffectPort);
    
    // Defining left climber motor & config
    leftClimber = new WPI_TalonSRX(Constants.Climber.leftClimberMotorPort);
    leftClimber.setInverted(true);
    leftClimber.setNeutralMode(NeutralMode.Brake);

    // Defining right climber motor & config
    rightClimber = new WPI_TalonSRX(Constants.Climber.rightClimberMotorPort);
    rightClimber.setInverted(true);
    rightClimber.setNeutralMode(NeutralMode.Brake);

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

  public void setLeftClimberSpeed(double climbSpeed) {
    // restricting climbSpeed to range -1 to 1
    if (climbSpeed > 1) {
      climbSpeed = 1; 
    } else if (climbSpeed < -1){
      climbSpeed = -1; 
    }

    // setting motor to speed climbSpeed
    leftClimber.set(climbSpeed);
  }

  public void setRightClimberSpeed(double climbSpeed) {
    // restricting climbSpeed to range -1 to 1
    if (climbSpeed > 1) {
      climbSpeed = 1; 
    } else if (climbSpeed < -1){
      climbSpeed = -1; 
    }

    // setting motor to speed climbSpeed
    rightClimber.set(climbSpeed);
  }

  /***
   * Sets the left and right climber motors to a specific voltage
   * @param climbSpeed Motor voltage values between -1 and 1 inclusive
   */
  public void setClimberSpeed(double climbSpeed) {
    // climber mechanism has 2 climbers, so set both to given speed
    setLeftClimberSpeed(climbSpeed);
    setRightClimberSpeed(climbSpeed);
  }

  public void stopClimber() {
    // stop motor --> set speed to 0
    setClimberSpeed(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
 }

// CLEANED