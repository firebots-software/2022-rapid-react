// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class DriveForward extends CommandBase {
  private final Drivetrain drivetrain;
  private double speed;
  public DriveForward() {
    this.drivetrain = Drivetrain.getInstance();
    this.speed = speed;

  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.PIDarcadeDrive(speed, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  public Set<Subsystem> getRequirements() {
    return Set.of(drivetrain);
  }
}
