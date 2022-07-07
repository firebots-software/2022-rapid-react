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
    // moves turret towards the left side first, until it reaches the soft stop
    if (!reachedLeftStop) {
      turret.setMotorSpeed(-Constants.Turret.constantTurretTurnSpeed);
      if (turret.getEncoderValDegrees() <= -80) {
        reachedLeftStop = true; 
        turret.stopMotor();
      }
    } else if (!reachedRightStop) { // once it reaches the left soft stop, start moving it towards the right soft stop
      turret.setMotorSpeed(Constants.Turret.constantTurretTurnSpeed);
      if (turret.getEncoderValDegrees() >= 80) {
        reachedRightStop = true; 
        turret.stopMotor();
      }
    } // along its path the target should enter it's field of vision, and then it should start aligning
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.getTv(); // once it sees the target, the command ends
  }
}
