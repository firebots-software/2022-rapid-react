// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DoNothing;
import frc.robot.commands.auton.DriveForTime;

public class DriveAndPause extends SequentialCommandGroup {
  /**
   * Drive for time, then pause for time.
   * 
   * @param speed = drivetrain percent output, front-back velocity [-1, 1]
   * @param driveTime = time to drive, in seconds
   * @param pauseTime = time to pause, in seconds
   */
    public DriveAndPause(double speed, double driveTime, double pauseTime) {
    addCommands(new DriveForTime(speed, driveTime),
                new DoNothing().withTimeout(pauseTime));
  }
}
