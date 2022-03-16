// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DriveForTime;
import frc.robot.commands.auton.TurnForTime;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiTurnShoot extends SequentialCommandGroup {
  /** Creates a new TaxiTurnShoot. */
  public TaxiTurnShoot() {
    addCommands(new DriveForTime(-0.5, 1),
                new TurnForTime(0.35, 3),
                new AimAndShoot());
  }
}
