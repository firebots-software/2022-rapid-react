// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class RunRoller extends CommandBase {

  private final Shooter shooter;
  
  /** Creates a new LoadBall. */
  public RunRoller() {
    this.shooter = Shooter.getInstance();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setRollerMotorSpeed(Constants.Shooter.maxRollerSpeed); //set to 0.5 for now 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopRollerMotor(); // check if we want to keep it running or not
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
