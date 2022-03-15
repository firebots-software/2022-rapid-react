// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretPositionControl extends CommandBase {
  private Turret turret;
  private double degreesToTurn, initialDegrees;
  private final double THRESHOLD = 0.25;
  
  /** Creates a new TurretPositionControl. */
  public TurretPositionControl(double degrees) {
    turret = Turret.getInstance();
    this.degreesToTurn = degrees;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initialDegrees = turret.getEncoderValDegrees();
    turret.setTurretPosition(degreesToTurn);
    System.out.println("starting position control");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ending position control");
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(turret.getMotionMagicPosition()) > Math.abs(degreesToTurn / 6.0);
    return Math.abs(turret.getEncoderValDegrees() - initialDegrees) > Math.abs(degreesToTurn);

    // return Math.abs(turret.getEncoderValDegrees() - targetDegrees) < THRESHOLD;
  }
}
