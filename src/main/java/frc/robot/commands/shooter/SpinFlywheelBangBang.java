// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SpinFlywheelBangBang extends CommandBase {
  private Shooter shooter;
  private BangBangController topBang, bottomBang;
  
  /** Creates a new SpinFlywheelBangBang. */
  public SpinFlywheelBangBang() {
    this.shooter = Shooter.getInstance();
    this.topBang = new BangBangController();
    this.bottomBang = new BangBangController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topOut = topBang.calculate(shooter.getTopShooterRPM(), shooter.getTopTargetRPM());
    shooter.setTopMotorSpeed(topOut);

    double bottomOut = bottomBang.calculate(shooter.getBottomShooterRPM(), shooter.getBottomTargetRPM());
    shooter.setBottomMotorSpeed(bottomOut);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopBothMotors(); // DO WE WANT THIS?
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean done = topBang.atSetpoint() && bottomBang.atSetpoint();
    SmartDashboard.putBoolean("flywheel bang bang done", done);
    return done;
  }
}
