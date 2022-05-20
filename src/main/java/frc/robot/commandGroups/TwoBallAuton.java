// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.TurnForAngle;
import frc.robot.commands.auton.TurnForTime;
import frc.robot.commands.limelight.ScanField;

public class TwoBallAuton extends SequentialCommandGroup {

  /**
   * Align robot manually to extra ball, taxi off tarmac while intaking, then turn around and shoot.
   * Uses turn for time -- simplified version of command used at comp. 
   */
  public TwoBallAuton() {
    addCommands(new DriveAndIntake(0.5, 1, 1).withTimeout(2),
                new TurnForTime(0.4, 1.6),
                new ScanField(),
                new AimAndShoot()
                );
  }
}
