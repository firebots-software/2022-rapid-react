// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StopShooter extends CommandBase {
  private Shooter shooter;

  public StopShooter() {
    shooter = Shooter.getInstance();
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stopBothMotors();
    shooter.stopRollerMotor();
    // System.out.println("stopping shooter");
  }

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