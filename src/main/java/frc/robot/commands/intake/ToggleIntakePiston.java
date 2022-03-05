// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.Set;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ToggleIntakePiston extends CommandBase {
  private Intake intake;
  
  /** Creates a new ToggleIntakePiston. */
  public ToggleIntakePiston() {
    this.intake = Intake.getInstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.togglePiston();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  // @Override
  // public Set<Subsystem> getRequirements() {
  //   return Set.of(intake);
  // }
}
