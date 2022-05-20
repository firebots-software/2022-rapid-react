// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.Constants;

public class ClimbToMax extends ClimbToHeight {

  /**
   *  Extend climbers to their max height. WARNING: DOES NOT WORK ON KETO BECAUSE WE
   * DO NOT HAVE ENCODERS ON THE CLIMBER MOTORS
   */
  public ClimbToMax() {
    super(Constants.Climber.climbSpeedUp, Constants.Climber.maxClimberHeight);
  }

  @Override
  public void initialize() {
    
  }

}
