// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.climber.StopClimber;
import frc.robot.commands.drivetrain.StopDrivetrain;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StopShooter;

public class StopEverything extends ParallelCommandGroup {
  public StopEverything() {
    addCommands(new StopShooter(),
                new StopIntake(),
                new StopDrivetrain(),
                new StopClimber());
  }
}

// CLEANED