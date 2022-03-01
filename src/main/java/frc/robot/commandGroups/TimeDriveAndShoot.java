// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DriveForTime;
import frc.robot.commands.auton.TurnForAngle;

public class TimeDriveAndShoot extends SequentialCommandGroup {
  public TimeDriveAndShoot() {
    addCommands(new DriveForTime(-0.5, 4),
                new TurnForAngle(180),
                new AimAndShoot());
  }
}
