// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class RetractClimber extends ClimbToHeight {
  /** Creates a new RetractClimber. */
  public RetractClimber() {
    super(-Constants.Climber.globalClimbSpeed, -1);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (climber.climberAtMax()) {
      setTargetHeight(Constants.Climber.middleBarHeight);
    } else {
      setTargetHeight(Constants.Climber.retractedHeight);
    }
  }
}
