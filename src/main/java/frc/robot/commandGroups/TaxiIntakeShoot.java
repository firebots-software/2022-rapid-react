// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DriveForTime;
import frc.robot.commands.auton.TurnForAngle;
import frc.robot.commands.intake.StartIntakeNoStop;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.TurnTurretToAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiIntakeShoot extends SequentialCommandGroup {
  /** Creates a new TaxiIntakeShoot. */
  public TaxiIntakeShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new StartIntakeNoStop(),
                new DriveForTime(-0.5, 4),
                new StopIntake(),
                new TurnForAngle(180),
                new AimAndShoot(),
                new TurnTurretToAngle(-45));
  }
}
