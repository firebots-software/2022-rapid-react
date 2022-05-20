// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SpinUpShooterNoStop;
import frc.robot.commands.shooter.StopShooter;

public class SpinUpAndShoot extends SequentialCommandGroup {

/**
 * Spin flywheels up to proper RPM, then use ball manipulator to deploy. Stop mechanism after shooting
 */
  public SpinUpAndShoot() {
    addCommands(new SpinUpShooterNoStop().withTimeout(3),
                new RunSpaghetAndRoll().withTimeout(3),
                new StopShooter());  }
}
