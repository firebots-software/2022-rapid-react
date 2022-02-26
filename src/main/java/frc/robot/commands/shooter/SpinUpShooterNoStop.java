// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

public class SpinUpShooterNoStop extends SpinUpShooter {

  /** Creates a new SpinUpShooter. */
  public SpinUpShooterNoStop() {
    super();
  }

  @Override
  public void end(boolean interrupted) {
      // override original, don't stop motors
  }
}
