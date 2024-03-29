// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.limelight.LimelightAimPosControl;
import frc.robot.commands.shooter.SpinUpShooterNoStop;
import frc.robot.commands.shooter.StopShooter;

public class AimAndShoot2Balls extends SequentialCommandGroup {
  /** Creates a new AimAndShoot2Balls. */
  public AimAndShoot2Balls() {
    addCommands(new LimelightAimPosControl(),
                new SpinUpShooterNoStop().withTimeout(3),
                new RunSpaghetAndRoll().withTimeout(3),
                new SpinUpShooterNoStop().withTimeout(3),
                new RunSpaghetAndRoll().withTimeout(3),
                new StopShooter());
  }
}
