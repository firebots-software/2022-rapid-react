// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ToggleAdjustableShooter extends CommandBase {
  private Shooter shooter;

  /**
   * Toggle whether shooter is using limelight feedback to adjust RPM or just
   * spinning at fixed, constant value
   */
  public ToggleAdjustableShooter() {
    this.shooter = Shooter.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setAdjustingRPM(!shooter.isRPMAdjusting());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
