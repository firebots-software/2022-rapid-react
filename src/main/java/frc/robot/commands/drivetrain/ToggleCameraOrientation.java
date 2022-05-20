// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ToggleCameraOrientation extends CommandBase {
  private Drivetrain drivetrain;

  /**
   * Toggle which camera feed is currently active
   */
  public ToggleCameraOrientation() {
    this.drivetrain = Drivetrain.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setUsingFrontCam(!drivetrain.isUsingFrontCam());
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
