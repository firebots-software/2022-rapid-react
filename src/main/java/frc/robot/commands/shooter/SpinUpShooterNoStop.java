// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinUpShooterNoStop extends SpinUpShooter {
  /** Creates a new FlywheelFalconFFNoStop. */
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
