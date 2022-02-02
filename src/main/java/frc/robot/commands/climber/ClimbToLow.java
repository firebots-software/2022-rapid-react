// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

import java.util.Collections;
import java.util.Set;


public class ClimbToLow extends CommandBase {
  public Climber climber;
  public double ClimberSpeed = 1;

  public double maxLowHeight = 50;

  public ClimbToLow() {
    this.climber = Climber.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
   if (ClimberSpeed < 1) {
     ClimberSpeed = 1;
   } else {
     ClimberSpeed = -1;
   }
   */
  }

  public void execute() {
    climber.climbToHangar(Constants.Climber.globalClimbSpeed, maxLowHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return Collections.emptySet();
    // do not require drivetrain here
  }
}
