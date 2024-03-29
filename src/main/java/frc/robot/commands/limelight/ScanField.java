// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class ScanField extends CommandBase {
  private Limelight limelight; 
  private Turret turret; 
  private boolean reachedLeftStop;
  private boolean reachedRightStop;

  /** Creates a new ScanField. */
  public ScanField() {
    limelight = Limelight.getInstance(); 
    turret = Turret.getInstance(); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reachedLeftStop = false;
    reachedRightStop = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!reachedLeftStop) {
      turret.setMotorSpeed(-Constants.Turret.constantTurretTurnSpeed);
      if (turret.getEncoderValDegrees() <= -80) {
        reachedLeftStop = true; 
        turret.stopMotor();
      }
    } else if (!reachedRightStop) {
      turret.setMotorSpeed(Constants.Turret.constantTurretTurnSpeed);
      if (turret.getEncoderValDegrees() >= 80) {
        reachedRightStop = true; 
        turret.stopMotor();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return limelight.getTx() < Constants.Turret.pidPositionToleranceDegrees;
    return limelight.getTv(); 
  }
}
