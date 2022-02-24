// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import java.sql.Driver;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.RamseteGenerator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiAndIntake extends SequentialCommandGroup {
  private Drivetrain drivetrain; 
  private Intake intake; 
  /** Creates a new MoveAndCollectBall. */
  public TaxiAndIntake() {
    drivetrain = Drivetrain.getInstance(); 
    intake = Intake.getInstance(); 
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(RamseteGenerator.generateCommandForPath(Paths.moveToBall), new TurnForAngle(180), intake.intakeBall());
  }
}
