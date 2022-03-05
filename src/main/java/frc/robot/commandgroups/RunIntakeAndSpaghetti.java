// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.intake.RunIntakeMotor;
import frc.robot.commands.intake.RunSpaghettiWheels;

public class RunIntakeAndSpaghetti extends ParallelCommandGroup {
  /** Creates a new RunIntakeAndSpaghetti. */

  public RunIntakeAndSpaghetti(boolean reversed) {
    if (!reversed) {
    addCommands(new RunIntakeMotor(Constants.Intake.INTAKE_SPEED_FORWARDS),
                new RunSpaghettiWheels(Constants.Intake.SPAGHETTI_SPEED));
    } else {
    addCommands(new RunIntakeMotor(-Constants.Intake.INTAKE_SPEED_FORWARDS),
                new RunSpaghettiWheels(-Constants.Intake.SPAGHETTI_SPEED));
    }
  }
}
