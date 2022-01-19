// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static Climber instance;

  /** Creates a new Climber. */
  private Climber() {}

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
