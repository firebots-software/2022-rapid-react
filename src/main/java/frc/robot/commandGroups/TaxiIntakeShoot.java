// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DriveForTime;
import frc.robot.commands.auton.TurnForAngle;
import frc.robot.commands.intake.StartIntakeNoStop;
import frc.robot.commands.intake.StopIntake;

public class TaxiIntakeShoot extends SequentialCommandGroup {

  /**
   * Align robot to additional ball, taxi off tarmac while intaking. Shoot 2 balls.
   * Uses PID to turn 180 degrees -- not used in comp
   */
  public TaxiIntakeShoot() {
    addCommands(new StartIntakeNoStop(),
                new DriveForTime(-0.5, 1),
                new StopIntake(),
                new TurnForAngle(180),
                new AimAndShoot());
  }
}
