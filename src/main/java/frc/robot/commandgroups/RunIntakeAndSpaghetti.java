// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.RunIntakeMotor;
import frc.robot.commands.intake.RunSpaghettiWheels;

public class RunIntakeAndSpaghetti extends ParallelCommandGroup {
  /** Creates a new RunIntakeAndSpaghetti. */
  public RunIntakeAndSpaghetti(double speed) {
    addCommands(new RunIntakeMotor(speed),
                new RunSpaghettiWheels(speed));
  }
}
