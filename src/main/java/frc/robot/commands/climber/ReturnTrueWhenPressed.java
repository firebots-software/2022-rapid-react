// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ReturnTrueWhenPressed extends CommandBase {
  private boolean isPressed;

  public ReturnTrueWhenPressed() {
    
  }


  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isPressed = true;
    SmartDashboard.putBoolean("isPressed", isPressed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isPressed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
