// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

import java.util.Collections;
import java.util.Set;

/**
 *  Keto currently does not have functioning encoders on its climber motors, so this command cannot run
 */

public class ClimbToHeight extends CommandBase {

  protected Climber climber;

  /***
   * Percent output has values between -1 and 1 inclusive and determines direction of motor
   * Target height is the target height in centimeters that the climber wants to go to
   */
  private double percentOutput, targetHeight;

  public ClimbToHeight(double percentOutput, double targetHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = Climber.getInstance();
    this.percentOutput = percentOutput;
    setTargetHeight(targetHeight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftHeight = climber.getLeftHeight();
    double rightHeight = climber.getRightHeight();

    if (leftHeight != targetHeight) {
      if (targetHeight > leftHeight) percentOutput = Math.abs(percentOutput);
      if (targetHeight < leftHeight) percentOutput = -Math.abs(percentOutput);
      climber.setLeftClimberSpeed(percentOutput);
    } else {
      climber.setLeftClimberSpeed(0);
    }

    if (rightHeight != targetHeight) {
      if (targetHeight > rightHeight) percentOutput = Math.abs(percentOutput);
      if (targetHeight < rightHeight) percentOutput = -Math.abs(percentOutput);
      climber.setRightClimberSpeed(percentOutput);
    } else {
      climber.setRightClimberSpeed(0);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(climber.getLeftHeight() - targetHeight) < Constants.Climber.encoderErrorRange && 
          Math.abs(climber.getRightHeight() - targetHeight) < Constants.Climber.encoderErrorRange;
  }

  @Override
  public Set<Subsystem> getRequirements() {
      return Set.of(climber);
  }

  protected void setTargetHeight(double height) {
    targetHeight = height;
  }

}
