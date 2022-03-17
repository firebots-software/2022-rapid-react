// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class FlywheelFalconFF extends CommandBase {
  protected Shooter shooter;
  protected Limelight limelight; 

  /** Creates a new FlywheelFalconFF. */
  public FlywheelFalconFF() {
    shooter = Shooter.getInstance();
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("starting falcon closed loop");
    shooter.setDistance(limelight.getDistanceToTarget());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setTopClosedLoopVelocity(shooter.getTopTargetRPM(limelight.getDistanceToTarget()));
    shooter.setBottomClosedLoopVelocity(shooter.getBottomTargetRPM(limelight.getDistanceToTarget()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopBothMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
