// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ChangeShooterTargetRPMTesting extends CommandBase {
  private Shooter shooter;
  private double change;
  private boolean changeTopMotor;
  
  /** Creates a new ChangeShooterTargetRPM. */
  public ChangeShooterTargetRPMTesting(double changeInRPM) {
    shooter = Shooter.getInstance();
    this.change = changeInRPM;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setBottomTestTargetRPM(shooter.getBottomTestTargetRPM() + change);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
