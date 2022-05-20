// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

public class SpinUpShooterNoStop extends SpinUpShooter {

  /**
   *  Spin flywheel up to fixed RPM, do not stop when command ends. Used in command groups.
   */
  public SpinUpShooterNoStop() {
    super();
  }

  @Override
  public boolean isFinished() {
    return shooter.atTargetRPM();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("spin up shooter done");
    // don't stop the motor
  }

 
}
