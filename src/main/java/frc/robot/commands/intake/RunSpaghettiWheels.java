// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunSpaghettiWheels extends CommandBase {
  private Intake intake;
  private double speed;

  /** Creates a new RunSpaghettiWheels. */
  public RunSpaghettiWheels(double speed) {
    this.intake = Intake.getInstance();
    this.speed = speed;
  }

  public RunSpaghettiWheels() { // default to predefined constant speed
    this.intake = Intake.getInstance();
    this.speed = Constants.Intake.SPAGHETTI_SPEED;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runSpaghettiMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// CLEANED