// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class FlywheelFalconFF extends CommandBase {
  private Shooter shooter;
  private double targetRPM;
  private static final double TOP_FLYWHEEL_CONST = 1;

  /** Creates a new FlywheelFalconFF. */
  public FlywheelFalconFF(double rpm) {
    shooter = Shooter.getInstance();
    targetRPM = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("starting falcon closed loop");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setTopClosedLoopVelocity(targetRPM * TOP_FLYWHEEL_CONST);
    shooter.setBottomClosedLoopVelocity(targetRPM);
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
