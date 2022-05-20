// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.Constants;

public class RetractClimber extends ClimbToHeight {

  /**
   * Automatically retract climber. If currently at max, retract to middle rung.
   * If at middle rung or below, retract completely. WARNING: DOES NOT WORK ON
   * KETO BECAUSE WE DO NOT HAVE ENCODERS ON THE CLIMBER MOTORS
   */
  public RetractClimber() {
    super(-Constants.Climber.climbSpeedUp, -1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (climber.climberAtMax()) {
      setTargetHeight(Constants.Climber.middleBarHeight);
    } else {
      setTargetHeight(0);
    }
  }
}
