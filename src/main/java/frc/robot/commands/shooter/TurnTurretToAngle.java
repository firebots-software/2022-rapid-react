// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;


public class TurnTurretToAngle extends TurnTurretForAngle {

  /** Creates a new TurnTurretToAngle. */
  public TurnTurretToAngle(double targetAngle) {
    super(targetAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(targetAngle);
  }

 
}
