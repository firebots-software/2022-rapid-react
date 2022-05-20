// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ZeroTurret extends CommandBase {
  private Turret turret;
  private double speed;
  private double DEFAULT_SPEED = 0.3;

  /** Creates a new ZeroTurret. */
  public ZeroTurret() {
    turret = Turret.getInstance();
    this.speed = DEFAULT_SPEED;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (turret.getEncoderValDegrees() > 0) {
      speed = -DEFAULT_SPEED;
    } else {
      speed = DEFAULT_SPEED;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setMotorSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.isHallEffectEnabled();
  }
}
