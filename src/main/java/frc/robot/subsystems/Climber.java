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
  
  private static double leftEncoderVal, rightEncoderVal;


  private final WPI_TalonSRX leftClimber, rightClimber;
  private static DigitalInput rightHallEffect, leftHallEffect;
  private static boolean leftHallEffectVal, rightHallEffectVal;

  private static MotorControllerGroup climber;




  private Climber() {
    rightHallEffect = new DigitalInput(Constants.Climber.arbitraryPortNum);
    leftHallEffect = new DigitalInput(Constants.Climber.arbitraryPortNum);
    
    this.leftClimber = new WPI_TalonSRX(Constants.Climber.arbitraryPortNum);
    this.rightClimber = new WPI_TalonSRX(Constants.Climber.arbitraryPortNum);

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
  
  public void climbToHangar(double climbSpeed, double height) {

    leftEncoderVal = leftClimber.getSelectedSensorPosition();
    rightEncoderVal = rightClimber.getSelectedSensorPosition();

    leftHallEffectVal = leftHallEffect.get();
    rightHallEffectVal = rightHallEffect.get();

    if (leftEncoderVal < height && climbSpeed > 0) {
      leftClimber.set(climbSpeed);
    } else if (leftEncoderVal > 0 && climbSpeed < 0) {
      leftClimber.set(climbSpeed);
    } else {
      leftClimber.stopMotor();
    }

    if (rightEncoderVal < height && climbSpeed > 0) {
      rightClimber.set(climbSpeed);
    } else if (leftEncoderVal > 0 && climbSpeed < 0) {
      rightClimber.set(climbSpeed);
    } else {
      rightClimber.stopMotor();
    }

    if (leftHallEffectVal && climbSpeed < 0 && leftEncoderVal < 40) {
      leftEncoderVal = 0;
      leftClimber.stopMotor();
    }

    if (leftHallEffectVal && climbSpeed > 0 && leftEncoderVal > 300) {
      leftEncoderVal = 330;
      leftClimber.stopMotor();
    }

    if (rightHallEffectVal && climbSpeed < 0 && leftEncoderVal < 40) {
      rightEncoderVal = 0;
      rightClimber.stopMotor();
    }

    if (rightHallEffectVal && climbSpeed > 0 && leftEncoderVal > 300) {
      rightEncoderVal = 330;
      rightClimber.stopMotor();
    }
  }

  public void setClimb (double climbSpeed) {

    leftClimber.set(climbSpeed);
    rightClimber.set(climbSpeed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
