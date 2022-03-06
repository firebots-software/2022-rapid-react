// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DriveForTime;
import frc.robot.commands.shooter.SpinUpShooter;
import frc.robot.commands.shooter.SpinUpShooterNoStop;
import frc.robot.commands.shooter.StopShooter;

public class DummyAutonAndShoot extends SequentialCommandGroup {
  public DummyAutonAndShoot() {
    addCommands(new DriveForTime(0.2, 1.8), 
                new SpinUpShooterNoStop().withTimeout(2.5),
                new RunSpaghetAndRoll().withTimeout(7),
                new StopShooter());
  }
}
                