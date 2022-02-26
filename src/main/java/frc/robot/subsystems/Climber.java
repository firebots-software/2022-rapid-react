// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static Climber instance;
  
  private final int CLIMBERSPEED = 1;


  private final WPI_TalonFX leftClimber, rightClimber;

  private static MotorControllerGroup climber;



  private Climber() {
    this.leftClimber = new WPI_TalonFX(Constants.Climber.leftClimberPort);
    this.rightClimber = new WPI_TalonFX(Constants.Climber.rightClimberPort);

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
  
  public void climbToMiddle() {
    climber.set(CLIMBERSPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
