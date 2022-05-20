// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurnTurretDegreesPerSecond extends CommandBase {
  private Turret turret;
  private double speed;

  /**
   * Experimental: use falcon velocity control to rotate turret at fixed angular
   * velocity
   * 
   * @param degPerS = turret velocity, in degrees per second, + is clockwise
   */
  public TurnTurretDegreesPerSecond(double degPerS) {
    turret = Turret.getInstance();
    this.speed = degPerS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setTurretClosedLoopVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
