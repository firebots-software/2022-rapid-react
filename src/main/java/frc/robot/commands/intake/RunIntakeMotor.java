// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunIntakeMotor extends CommandBase {
  private Intake intake;
  private double speed;
  
  /** Creates a new RunIntakeMotor. */
  public RunIntakeMotor(double speed) {
    this.intake = Intake.getInstance();
    this.speed = speed;
  }

  public RunIntakeMotor() {
    this.intake = Intake.getInstance();
    this.speed = Constants.Intake.INTAKE_SPEED_FORWARDS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // intake.extendIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runRollerMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotors();
    // intake.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
