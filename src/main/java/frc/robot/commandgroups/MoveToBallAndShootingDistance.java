// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.RamseteGenerator;
import frc.robot.commands.auton.TurnForAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToBallAndShootingDistance extends SequentialCommandGroup {

  /** Creates a new MoveFurthestDistance. */
  public MoveToBallAndShootingDistance() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(RamseteGenerator.generateCommandForPath(Paths.moveToBall), 
                new TurnForAngle(180), 
                RamseteGenerator.generateCommandForPath(Paths.moveToShootingDistanceFromBall));
  }
}
