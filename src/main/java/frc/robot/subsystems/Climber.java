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
  private static DigitalInput bottomHallEffect = new DigitalInput(Constants.Climber.arbitraryPortNum);
  private static DigitalInput topHallEffect = new DigitalInput(Constants.Climber.arbitraryPortNum);

  private static MotorControllerGroup climber;




  private Climber() {
    this.leftClimber = new WPI_TalonSRX(Constants.Climber.arbitraryPortNum);
    this.rightClimber = new WPI_TalonSRX(Constants.Climber.arbitraryPortNum);
    leftEncoderVal = leftClimber.getSelectedSensorPosition();
    rightEncoderVal = rightClimber.getSelectedSensorPosition();

    climber = new MotorControllerGroup(leftClimber, rightClimber);
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
  
  public void climbToHangar(int climbSpeed, double height) {
    leftEncoderVal = leftClimber.getSelectedSensorPosition();
    rightEncoderVal = rightClimber.getSelectedSensorPosition();

    if (leftEncoderVal < height && rightEncoderVal < height && climbSpeed == 1) {
      climber.set(climbSpeed);
    } else if (leftEncoderVal > 0 && rightEncoderVal > 0 && climbSpeed == -1) {
      climber.set(climbSpeed);
    } else {
      climber.stopMotor();
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
