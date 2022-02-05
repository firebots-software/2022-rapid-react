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

public class ClimbToHeight extends CommandBase {

  private final Climber climber;

  /***
   * Motor voltage has values between -1 and 1 inclusive and determines direction of motor
   * Target height is the target height in centimeters that the climber wants to go to
   */
  private double motorVoltage, targetHeight;

  public ClimbToHeight(double motorVoltage, double targetHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = Climber.getInstance();
    this.motorVoltage = motorVoltage;
    this.targetHeight = targetHeight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftHeight = climber.getLeftHeight();
    double rightHeight = climber.getRightHeight();

    boolean leftHallEffectVal = climber.getLeftHallEffectValue();
    boolean rightHallEffectVal = climber.getRightHallEffectValue();

    if (leftHeight < this.targetHeight && motorVoltage > 0) {
      climber.setLeftClimberSpeed(motorVoltage);
    } else if (leftHeight > 0 && motorVoltage < 0) {
      climber.setLeftClimberSpeed(motorVoltage);
    } else {
      climber.setLeftClimberSpeed(0);
    }

    if (rightHeight < targetHeight && motorVoltage > 0) {
      climber.setRightClimberSpeed(motorVoltage);
    } else if (rightHeight > 0 && motorVoltage < 0) {
      climber.setRightClimberSpeed(motorVoltage);
    } else {
      climber.setRightClimberSpeed(0);
    }

    if(leftHallEffectVal && leftHeight < Constants.Climber.encoderErrorRange && motorVoltage < 0) {
      climber.setLeftClimberSpeed(0);
      climber.setLeftHeight(0);
    }
    if(leftHallEffectVal && leftHeight > Constants.Climber.maxClimberHeight - Constants.Climber.encoderErrorRange && motorVoltage > 0) {
      climber.setLeftClimberSpeed(0);
      climber.setLeftHeight(Constants.Climber.maxClimberHeight);
    }

    if(rightHallEffectVal && rightHeight < Constants.Climber.encoderErrorRange && motorVoltage < 0) {
      climber.setRightClimberSpeed(0);
      climber.setRightHeight(0);
    }
    if(rightHallEffectVal && rightHeight > Constants.Climber.maxClimberHeight - Constants.Climber.encoderErrorRange && motorVoltage > 0) {
      climber.setRightClimberSpeed(0);
      climber.setRightHeight(Constants.Climber.maxClimberHeight);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
