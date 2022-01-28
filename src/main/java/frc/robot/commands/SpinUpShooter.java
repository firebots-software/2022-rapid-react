// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {
  private final Shooter shooter;
  /** Creates a new SpinUpShooter. */
  public SpinUpShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = Shooter.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double current = shooter.getRPM();
    //double error = desired - current; // desired = desired aimed position we want
    //double newVal = P * (error); //magic number P (proportionality constant)
   // shooter.setVal(newVal); //
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
