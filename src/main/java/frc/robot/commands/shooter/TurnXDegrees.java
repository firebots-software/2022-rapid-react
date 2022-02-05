// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

// manual alignment of robot with hub 
public class TurnXDegrees extends CommandBase {
  private Turret turret;
  private double targetAngle;
  private PIDController pid;

  // pid constants 
  private double kP = 0.5;
  private double kI = 0;
  private double kD = 0;
  
  public TurnXDegrees(double targetAngle) {
    turret = Turret.getInstance();
    this.targetAngle = targetAngle;

    this.pid = new PIDController(kP, kI, kD);
    pid.setSetpoint(targetAngle);
    pid.setTolerance(Constants.Turret.pidPositionToleranceDegrees, Constants.Turret.pidVelToleranceDegPerSecond);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.zeroEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = pid.calculate(turret.getEncoderValDegrees());
    turret.setMotorSpeed(pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
