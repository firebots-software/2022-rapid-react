// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.intake.RunSpaghettiWheels;
import frc.robot.commands.shooter.StartRoller;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunSpaghetAndRoll extends ParallelCommandGroup {

  Shooter shooter = Shooter.getInstance();
  Limelight limelight = Limelight.getInstance();


  /** Creates a new SpaghettiRollerShooter. */
  public RunSpaghetAndRoll() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StartRoller(), new RunSpaghettiWheels(Constants.Intake.SPAGHETTI_SPEED));
  }


  @Override
  public void end(boolean interrupted) {
    shooter.stopBothMotors();
    // limelight.setLedStatus(false);
    super.end(interrupted);
  }
}
