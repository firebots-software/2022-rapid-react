// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpinUpShooterNoStop extends SpinUpShooter {

  /** Creates a new SpinUpShooter. */
  public SpinUpShooterNoStop() {
    super();  
  }

  @Override
  public boolean isFinished() {
    boolean done = pidTop.atSetpoint() & pidBottom.atSetpoint();
    if (done) {
      timer.stop();
    }
    SmartDashboard.putBoolean("SHOOTER AT RPM", done);
    return false; // RETURN FALSE -- KEEP THE WHEELS SPINNING ONCE THEY'RE UP TO SPEED
  }
  
  @Override
  public void end(boolean interrupted) {
      // override original, don't stop motors
  }
}
