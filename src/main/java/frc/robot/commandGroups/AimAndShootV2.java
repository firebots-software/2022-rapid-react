// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.limelight.AlignToTargetPosControl;
import frc.robot.commands.shooter.FlywheelFalconFFNoStop;
import frc.robot.commands.shooter.StopShooter;

public class AimAndShootV2 extends SequentialCommandGroup {
  /** Creates a new AimAndShootV2. */
  public AimAndShootV2() {
    addCommands(new AlignToTargetPosControl(),
                new FlywheelFalconFFNoStop().withTimeout(3),
                new RunSpaghetAndRoll().withTimeout(5),
                new StopShooter());
  }
}
