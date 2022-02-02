// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;

public class LoadBall extends CommandBase {
  private final Shooter shooter;
  /** Creates a new LoadBall. */
  public LoadBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = Shooter.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.extendPiston();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.retractPiston();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

 
}
